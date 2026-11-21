#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/image_encodings.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>        //ROS和cv信息的桥梁
#include <opencv2/opencv.hpp>
#include <algorithm>   
#include <cmath>     
#include <array>  
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_broadcaster.h>
#include <visualization_msgs/MarkerArray.h>
#include <ft_detection/Detection.h>
#include <ft_detection/DetectionFrame.h>

class FtDetecor{
public:
    FtDetecor() : nh_(), pnh_("~"), it_(nh_){
        image_topic_ = pnh_.param<std::string>("image_topic","/hk_camera/image_raw/");
        camera_info_topic_ = pnh_.param<std::string>("camera_info_topic","/hk_camera/camera_info");
        transport_ = pnh_.param<std::string>("transport", "compressed");
        queue_size_ = pnh_.param<int>("queue_size", 2);
        debug_topic_ = pnh_.param<std::string>("debug_topic", "/ft/debug/image");

        // HSV 阈值：默认值集中在 loadColorConfig()，可被 yaml / 命令行覆盖
        loadColorConfig();

        // 灯条粗筛阈值：默认值集中在 loadBarFilterConfig()
        loadBarFilterConfig();

        w_big_ = pnh_.param<float>("w_big", 0.235f);
        h_big_ = pnh_.param<float>("l_big", 0.127f);

        object_points_big_ = {
            {-w_big_/2,  h_big_/2, 0.f},     //TL
            { w_big_/2,  h_big_/2, 0.f},     //TR
            { w_big_/2, -h_big_/2,  0.f},    //BR
            {-w_big_/2, -h_big_/2,  0.f}     //BL
        };

        marker_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("/ft/armor_markers", 10);
        det_pub_ = nh_.advertise<ft_detection::DetectionFrame>("/ft/detections", 10);

        // //放大图像
        // zoom_ = pnh_.param<double>("debug_zoom", 1.0);
        // // ROS_INFO("%lf", zoom_);

        //订阅相机参数
        camera_info_sub = nh_.subscribe(camera_info_topic_,queue_size_,&FtDetecor::cameraInfoCallBack,this);

        //用什么传输
        image_transport::TransportHints hints(transport_, ros::TransportHints().tcpNoDelay(), pnh_);
        try
        {
            image_sub_ = it_.subscribe(image_topic_,queue_size_,&FtDetecor::imageRawCallBack, this, hints);
        }
        //接住异常
        catch(const image_transport::TransportLoadException& e)
        {
            ROS_ERROR("subscribe failed: %s", e.what());
            throw;  //抛出
        }
        image_debug_ = it_.advertise(debug_topic_,queue_size_);
    }
private:
    struct armor{
        int size;       //0:小，1:大
        bool is_red;
        std::array<cv::Point2f, 4> points;
        cv::Mat rvec;
        cv::Mat tvec;
        double  mean_err;
    };

    // 一段 HSV 阈值：{下界 (H,S,V), 上界 (H,S,V)}
    using HsvRange = std::pair<cv::Scalar, cv::Scalar>;

    //相机内参回调函数
    void cameraInfoCallBack(const sensor_msgs::CameraInfoConstPtr &msg){
        //ROS_INFO("b_x=%d", msg->binning_x);
        if(msg->height == 0 || msg->width == 0 || msg->K[0] <= 0){
            ROS_WARN_THROTTLE(5.0, "invalid camera_info");
            return;
        }       //即使return了，camera_info_会缓存最新的

        //可以来个判断前后info是否一样来决定是否重建

        //K就是相机的内参矩阵
        K_ = (cv::Mat_<double>(3,3) <<
                msg->K[0], msg->K[1], msg->K[2],
                msg->K[3], msg->K[4], msg->K[5],
                msg->K[6], msg->K[7], msg->K[8]
        );

        //D就是畸变参数
        //.size返回的是int_t无符号
        //D_ 是一个 1×n 的 double 行向量
        D_  = cv::Mat(1,static_cast<int>(msg->D.size()),CV_64F);
         for(size_t i=0; i < msg->D.size(); ++i){
            D_.at<double>(0,static_cast<int>(i)) = msg->D[i];
         }
        camera_info_ = msg;

    //     //test
    //      ROS_INFO("camera: %ux%u fx=%.2f fy=%.2f cx=%.2f cy=%.2f D=%zu frame=%s",
    //          msg->width, msg->height, K_.at<double>(0, 0), K_.at<double>(1, 1),
    //          K_.at<double>(0, 2), K_.at<double>(1, 2), msg->D.size(),
    //          msg->header.frame_id.c_str());
     }

    void imageRawCallBack(const sensor_msgs::ImageConstPtr &msg){
        if(!camera_info_){
            ROS_WARN("null");
            return;
            //应该要用yaml里面的
            /*code*/
        }

        // //test
        ROS_INFO("transporting susscessfully");
        //ROS_INFO("%s",msg->encoding.c_str());

        if(msg->data.empty()) return;
        if(msg->header.stamp.isZero()){
            ROS_WARN_THROTTLE(5.0, "image stamp is zero, drop frame");      //节流
            return;
        }

        //msg->cv::Mat
        // bgr8 与消息 encoding 一致时 toCvShare 不拷贝；
        // 返回的 Mat 引用消息 buffer，只读，后面要画图必须 clone
        cv_bridge::CvImageConstPtr cv_ptr;
        try{
            cv_ptr = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::BGR8);
        }
        catch(const cv_bridge::Exception& e){
            ROS_ERROR_STREAM_THROTTLE(2.0, "cv_bridge: " << e.what());
            return;
        }
        const cv::Mat& bgr = cv_ptr->image;

        debug_ = bgr.clone();
        //=====装甲板识别=====
        //HSV
        cv::cvtColor(debug_,image_hsv_,cv::COLOR_BGR2HSV);

        //此处识别红色，不过灯条中间已经到橙色甚至黄色，地板倒影也有干扰
        // cv::inRange(image_hsv_,cv::Scalar(0, 100, 200),cv::Scalar(34, 255, 255),mask1_);
        // cv::inRange(image_hsv_,cv::Scalar(170, 100, 200),cv::Scalar(179, 255, 255),mask2_);
        // cv::bitwise_or(mask1_, mask2_, mask_);

        //扩增识别蓝色红色
        bool is_red = false;
        FtDetecor::colorMask(image_hsv_, red_ranges_, mask1_);
        FtDetecor::colorMask(image_hsv_, blue_ranges_, mask2_);
        if(cv::countNonZero(mask1_) > cv::countNonZero(mask2_)){
            is_red = true;
            mask_ = mask1_;
        }
        else{
            is_red = false;
            mask_ = mask2_;
        }

            //====画灯条轮廓====
        cv::findContours(mask_,contours_,cv::RETR_EXTERNAL,cv::CHAIN_APPROX_SIMPLE);
        //粗筛（不过视频挺好的，没什么干扰）
        //====要yaml，动态调参====
        int kept = 0;
        std::vector<LightBar> bars_;
        for(size_t i = 0; i < contours_.size(); ++i){
            const auto& contour = contours_[i];
            //面积过滤
            const double area = cv::contourArea(contour);
            if(area < bar_min_area_) continue;

            //长宽高过滤，有比值，筛出细长的
            // const cv::Rect r = cv::boundingRect(contour);
            // if(r.width <= 0) continue;
            // const double ratio = static_cast<double>(r.height) / r.width;
            // if(r.height < 20 || ratio < 2.0) continue;
            //旋转矩形（不受倾斜影响）
            const cv::RotatedRect rr = cv::minAreaRect(contour);
            const float long_side  = std::max(rr.size.width,  rr.size.height);
            const float short_side = std::min(rr.size.height, rr.size.width);
            if (short_side < 1.0f) continue;      // 防除零，不可配置

            // 长边下限（0 = 关闭）
            if (bar_min_long_side_ > 0.0 && long_side < bar_min_long_side_) continue;

            // 长宽比上下限（0 = 关闭）
            const double ratio = static_cast<double>(long_side) / short_side;
            if (bar_min_ratio_ > 0.0 && ratio < bar_min_ratio_) continue;
            if (bar_max_ratio_ > 0.0 && ratio > bar_max_ratio_) continue;

            // 填充率：排除 L 形、散点、锯齿（0 = 关闭）
            const double fill = area / static_cast<double>(long_side * short_side);
            if (bar_min_fill_ > 0.0 && fill < bar_min_fill_) continue;

             //RotatedRect::angle 的语义在 OpenCV 版本间变过

            // int    idx    = 0;
            // float  maxlen = 0.0f;
            // for (int k = 0; k < 4; ++k) {
            //     const float len = std::hypot(p[(k + 1) % 4].x - p[k].x,
            //                          p[(k + 1) % 4].y - p[k].y);
            //     if (len > maxlen) { maxlen = len; idx = k; }
            // }
            // const float  dx   = p[(idx + 1) % 4].x - p[idx].x;
            // const float  dy   = p[(idx + 1) % 4].y - p[idx].y;
            // const double tilt = std::atan2(std::fabs(dx), std::fabs(dy)) * 180.0 / CV_PI;
            const float tilt = FtDetecor::barAngleDeg(rr);
            if (bar_max_tilt_ > 0.0 && tilt > bar_max_tilt_) continue;   // 0 = 关闭

            cv::Point2f p[4];       //向量
            rr.points(p);
            

            ++kept;
            //通过,存灯条
            bars_.push_back({rr, rr.center, long_side, short_side, barAngleDeg(rr)});
            
            //画图
            //cv::rectangle(debug_,r,cv::Scalar(0,255,0),2);      //绿色
            //只能用 cv::line 画，不能用 cv::rectangle：后者只接受轴对齐的 cv::Rect，画不出旋转框
            for(int k = 0; k < 4; ++k){
                cv::line(debug_, p[k], p[(k+1)%4], cv::Scalar(0,255,0),2);
            }
            //cv::drawContours(debug_, contours_, static_cast<int>(i), cv::Scalar(255, 0, 0), 1);     //蓝色
        }
        cv::drawContours(debug_, contours_, -1, cv::Scalar(255, 0, 0), 1);     //蓝色
        
            //====灯条配对====
        int id = 0;
        std::vector<armor> armors_big;   // 每块装甲板的 4 角点（TL,TR,BR,BL）,定义1
        std::vector<armor> armors_small;
        for(size_t i = 0; i < bars_.size(); ++i){
            for(size_t j = i + 1; j < bars_.size(); ++j){
                const auto& a = bars_[i];
                const auto& b = bars_[j];
            

                float avg_len = (a.length + b.length) * 0.5f;

                //高度差(长边)
                //max_height_diff
                float h_diff = std::abs(a.length - b.length) / avg_len;
                if (h_diff > 0.3) continue;

                //角度差
                //max_angle_diff
                float a_diff = std::abs(a.angle - b.angle);
                if (a_diff > 5.0) continue;

                //中心高度差
                //max_dy
                float dy = std::abs(a.center.y - b.center.y) / avg_len;
                if (dy > 0.5) continue;

                //间距
                //min_gap_ratio，max_gap_ratio
                float gap = cv::norm(a.center - b.center);
                float gap_ratio = gap / avg_len;
                if (gap_ratio < 0.0 || gap_ratio > 4.5) continue;

                //test
                //ROS_INFO("1");

                cv::Point2f pa[4],pb[4];
                a.rect.points(pa);
                b.rect.points(pb);

                std::vector<cv::Point2f> all(pa,pa+4);
                all.insert(all.end(), pb, pb + 4);

                std::vector<cv::Point2f> hull;
                cv::convexHull(all,hull);
                if(hull.size() < 4) continue;

                //ROS_INFO("2");

                // ---------- 6) 凸包 → 4 边形 ----------
                std::vector<cv::Point2f> quad;
                const double eps = 0.03 * cv::arcLength(hull, true);
                cv::approxPolyDP(hull, quad, eps, true);
                if (quad.size() != 4) continue;

                //ROS_INFO("3");

                // ---------- 7) 排序成 TL, TR, BR, BL ----------
                const auto ordered = orderPointsByAngle(quad);
                if (ordered.size() != 4) continue;

                //ROS_INFO("4");

                // ---------- 8) 形状校验 ----------
                const float w = static_cast<float>(cv::norm(ordered[1] - ordered[0]));  // 上边长
                const float h = static_cast<float>(cv::norm(ordered[3] - ordered[0]));  // 左边长
                if (w < 1.0f || h < 1.0f) continue;

                const double ar = static_cast<double>(h) / w;      // 装甲板大致 1:1 ~ 1:2
                if (ar < 0.0 || ar > 2.5) continue;
                //初步观察ar0.3可以作为大小装甲板的阈值，但这个并不可靠，不过考核静态已经够用了，不过我怀疑动态误差会加大，其实可以来个，距离误差表
                int size = 0;
                if(ar > 0.3){
                    size = 1;
                }

                ROS_INFO("ar: %f",ar);

                //ROS_INFO("5");

                if (cv::contourArea(ordered) < 300.0) continue;


                // ---------- 9) 记录 + 画红框 ----------
                for (int k = 0; k < 4; ++k) {
                cv::line(debug_, ordered[k], ordered[(k + 1) % 4],
                     cv::Scalar(0, 0, 255), 2);            // 红框
                }
                const cv::Point2f ctr = (ordered[0] + ordered[1] + ordered[2] + ordered[3]) * 0.25f;
                cv::circle(debug_, ctr, 3, cv::Scalar(0, 255, 255), -1);   // 黄色中心点
                char txt[64];
                std::snprintf(txt, sizeof(txt), "ar=%.2f s=%d", ar, size);

                cv::putText(debug_, txt,
                            cv::Point(static_cast<int>(ctr.x) - 40, static_cast<int>(ctr.y) - 40),
                            cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 255), 1);


                //====位姿解算与TF====
                cv::Mat rvec, tvec;
                //这里应该要小的装甲板识别
                bool ok = cv::solvePnP(object_points_big_, ordered, K_, D_, rvec, tvec, false, cv::SOLVEPNP_IPPE);
                if(!ok){
                    ROS_WARN("solvePnP failed");
                    continue;
                }

                //旋转矩阵
                cv::Mat R;  
                cv::Rodrigues(rvec, R);

                // //四元数
                // tf2::Matrix3x3 m(
                //     R.at<double>(0,0), R.at<double>(0,1), R.at<double>(0,2),   // 第 0 行
                //     R.at<double>(1,0), R.at<double>(1,1), R.at<double>(1,2),   // 第 1 行
                //     R.at<double>(2,0), R.at<double>(2,1), R.at<double>(2,2)    // 第 2 行
                // );

                // tf2::Quaternion q;
                // m.getRotation(q);
                // q.normalize();

                // geometry_msgs::TransformStamped ts;
                // ts.header.stamp = msg->header.stamp;
                // ts.header.frame_id = msg->header.frame_id;
                // ts.child_frame_id = "arrmor_";
                // ++id;

                // ts.transform.translation.x = tvec.at<double>(0);
                // ts.transform.translation.y = tvec.at<double>(1);
                // ts.transform.translation.z = tvec.at<double>(2);
                // ts.transform.rotation      = tf2::toMsg(q);

                // tf_broadcaster_.sendTransform(ts);

                // double x = tvec.at<double>(0);
                // double y = tvec.at<double>(1);
                // double z = tvec.at<double>(2);
                // double distance = cv::norm(tvec);

                //=====验证====
                //选取
                std::vector<cv::Point2f> reproj;
                cv::projectPoints(object_points_big_, rvec, tvec, K_, D_, reproj);
                double sum = 0, max_err = 0;
                for (size_t i = 0; i < reproj.size(); ++i) {
                    double e = cv::norm(reproj[i] - ordered[i]);
                    sum += e;
                    max_err = std::max(max_err, e);
                    // cv::line(debug_, reproj[i], reproj[(i + 1) % 4],
                    //             cv::Scalar(255, 0, 255), 2);    
                }        
                double mean_err = sum / reproj.size();
                ROS_INFO("reproj: mean=%.2f px, max=%.2f px", mean_err, max_err);

                if(size == 0){
                    armors_small.push_back({size, is_red, {ordered[0], ordered[1], ordered[2], ordered[3]}, rvec, tvec, mean_err});
                }
                else{
                    armors_big.push_back({size, is_red, {ordered[0], ordered[1], ordered[2], ordered[3]}, rvec, tvec, mean_err});
                }
            }
        }

        //====位姿解算与TF====
        // /*一开始直接发布到tf，但tf并不适合表达动态变化的，所以我决定用MarkerArray
        //  *但我依然会采用最最优目标，不过这是为了更好预测
        // */
        // visualization_msgs::MarkerArray arr;

        FtDetecor::publishTfMsg(armors_small, msg, 0);
        FtDetecor::publishTfMsg(armors_big, msg, 1);
        publishDetections(armors_small, armors_big, msg->header);
        // if(armors.size() > 0){
        //     int id_small=0, id_big=0;
        //     for (size_t i = 0; i < armors.size(); ++i){
        //         cv::Mat rvec, tvec;
        //         bool ok = cv::solvePnP(object_points_big_, armors[i].points, K_, D_, rvec, tvec, false, cv::SOLVEPNP_IPPE);
        //         if(!ok){
        //             ROS_WARN("solvePnP failed");
        //             continue;
        //         }

        //         //旋转矩阵
        //         cv::Mat R;  
        //         cv::Rodrigues(rvec, R);

        //         //四元数
        //         tf2::Matrix3x3 m(
        //             R.at<double>(0,0), R.at<double>(0,1), R.at<double>(0,2),   // 第 0 行
        //             R.at<double>(1,0), R.at<double>(1,1), R.at<double>(1,2),   // 第 1 行
        //             R.at<double>(2,0), R.at<double>(2,1), R.at<double>(2,2)    // 第 2 行
        //         );

        //         tf2::Quaternion q;
        //         m.getRotation(q);
        //         q.normalize();

        //         geometry_msgs::TransformStamped ts;
        //         ts.header.stamp = msg->header.stamp;
        //         ts.header.frame_id = msg->header.frame_id;
        //         if(armors[i].size == 0){
        //             ts.child_frame_id = "arrmor_0_" + std::to_string(id_small);
        //             ++id_small;
        //         }
        //         else{
        //             ts.child_frame_id = "arrmor_1_" + std::to_string(id_big);
        //             ++id_big;
        //         }

        //         ts.transform.translation.x = tvec.at<double>(0);
        //         ts.transform.translation.y = tvec.at<double>(1);
        //         ts.transform.translation.z = tvec.at<double>(2);
        //         ts.transform.rotation      = tf2::toMsg(q);

        //         tf_broadcaster_.sendTransform(ts);
        //     }
        // }

        //=====发布图像=====
        // if (zoom_ != 1.0) {
        //     cv::resize(debug_, out_img_, cv::Size(), zoom_, zoom_,
        //        cv::INTER_NEAREST);     // ← 关键：最近邻，像素边界锐利
        // } else {
        //     out_img_ = debug_;
        // }
        cv_bridge::CvImage out(msg->header,sensor_msgs::image_encodings::BGR8,debug_);
        //cv_bridge::CvImage out(msg->header,sensor_msgs::image_encodings::MONO8,mask_);
        image_debug_.publish(out.toImageMsg());

        // test
        // cv::imshow("1",bgr);
        // cv::waitKey(1);
    }

    //HSV掩膜函数
    static void colorMask(const cv::Mat& hsv, const std::vector<HsvRange>& ranges, cv::Mat& mask){
        cv::Mat temp;
        mask.create(hsv.size(), CV_8UC1);
        mask.setTo(0);
        for(const auto& r : ranges){
            cv::inRange(hsv, r.first, r.second, temp);
            cv::bitwise_or(mask, temp, mask);
        }
    }

    // 读一段 HSV 阈值：prefix 形如 "red1"，对应 red1_h_lo / red1_s_lo / ... 共 6 个 key
    void loadRange(const std::string& prefix,
                   const cv::Scalar& def_lo,
                   const cv::Scalar& def_hi,
                   HsvRange& out)
    {
        auto g = [&](const char* suffix, double def) {
            return pnh_.param<double>(prefix + suffix, def);
        };

        out.first  = cv::Scalar(g("_h_lo", def_lo[0]), g("_s_lo", def_lo[1]), g("_v_lo", def_lo[2]));
        out.second = cv::Scalar(g("_h_hi", def_hi[0]), g("_s_hi", def_hi[1]), g("_v_hi", def_hi[2]));

        // 下界 > 上界会让掩膜恒为空，交换并告警
        for (int i = 0; i < 3; ++i) {
            if (out.first[i] > out.second[i]) {
                ROS_WARN("HSV 参数 %s 第 %d 通道：下界 %.0f > 上界 %.0f，已自动交换",
                         prefix.c_str(), i, out.first[i], out.second[i]);
                std::swap(out.first[i], out.second[i]);
            }
        }

        // 范围校验（H 只有 0~179，S/V 只有 0~255）
        if (out.first[0] < 0.0 || out.second[0] > 179.0) {
            ROS_WARN("HSV 参数 %s 的 H 超出 0~179，OpenCV 会截断", prefix.c_str());
        }
        if (out.second[1] > 255.0 || out.second[2] > 255.0) {
            ROS_WARN("HSV 参数 %s 的 S/V 超出 0~255，OpenCV 会截断", prefix.c_str());
        }
    }

    // 读取全部 HSV 阈值；参数未提供时使用内置默认值（与原硬编码一致）
    void loadColorConfig()
    {
        red_ranges_.clear();
        blue_ranges_.clear();

        red_ranges_.resize(2);
        loadRange("red1", cv::Scalar(  0, 100, 200), cv::Scalar( 34, 255, 255), red_ranges_[0]);
        loadRange("red2", cv::Scalar(170, 100, 200), cv::Scalar(179, 255, 255), red_ranges_[1]);

        blue_ranges_.resize(1);
        loadRange("blue", cv::Scalar(100, 120,  80), cv::Scalar(140, 255, 255), blue_ranges_[0]);

        for (size_t i = 0; i < red_ranges_.size(); ++i) {
            const auto& r = red_ranges_[i];
            ROS_INFO("HSV red%zu : (%.0f,%.0f,%.0f) ~ (%.0f,%.0f,%.0f)",
                     i + 1,
                     r.first[0],  r.first[1],  r.first[2],
                     r.second[0], r.second[1], r.second[2]);
        }
        for (size_t i = 0; i < blue_ranges_.size(); ++i) {
            const auto& r = blue_ranges_[i];
            ROS_INFO("HSV blue%zu: (%.0f,%.0f,%.0f) ~ (%.0f,%.0f,%.0f)",
                     i + 1,
                     r.first[0],  r.first[1],  r.first[2],
                     r.second[0], r.second[1], r.second[2]);
        }
    }

    // 读取灯条粗筛阈值；0 表示关闭该项检查（与旧的注释状态等价）
    void loadBarFilterConfig()
    {
        bar_min_area_      = pnh_.param<double>("bar_min_area",      10.0);
        bar_max_tilt_      = pnh_.param<double>("bar_max_tilt",      40.0);
        bar_min_long_side_ = pnh_.param<double>("bar_min_long_side",  0.0);
        bar_min_ratio_     = pnh_.param<double>("bar_min_ratio",      0.0);
        bar_max_ratio_     = pnh_.param<double>("bar_max_ratio",      0.0);
        bar_min_fill_      = pnh_.param<double>("bar_min_fill",       0.0);

        ROS_INFO("bar filter: area>=%.1f  tilt<=%.1f  long>=%.1f  ratio=[%.2f,%.2f]  fill>=%.2f  (0=off)",
                 bar_min_area_, bar_max_tilt_, bar_min_long_side_,
                 bar_min_ratio_, bar_max_ratio_, bar_min_fill_);
    }

    //返回的是竖直的夹角，0~90
    float barAngleDeg(const cv::RotatedRect& rr) {
        cv::Point2f p[4];
        rr.points(p);

        float best = 0.f, max_len = -1.f;
        for (int i = 0; i < 4; ++i) {
        cv::Point2f d = p[(i + 1) % 4] - p[i];
        float l = std::hypot(d.x, d.y);
        if (l > max_len) {
            max_len = l;
            best = std::atan2(d.y, d.x) * 180.f / static_cast<float>(CV_PI);
        }
        }
        while (best >= 90.f)  best -= 180.f;
        while (best < -90.f)  best += 180.f;
        //return best;
        return 90.f - std::fabs(best);
    }

    //返会排序好的点，左上 → 右上 → 右下 → 左下
    std::vector<cv::Point2f> orderPointsByAngle(std::vector<cv::Point2f> pts){
        if(pts.size() != 4) return {};

         // 用四点均值作为中心（比最小外接矩形中心更稳）
        cv::Point2f c(0.0f, 0.0f);
        for (const auto& p : pts) c += p;
        c *= 1.0f / static_cast<float>(pts.size());

        // 图像坐标 y 向下 → atan2 递增 = 视觉顺时针
        std::sort(pts.begin(), pts.end(), [c](const cv::Point2f& a, const cv::Point2f& b) {
            return std::atan2(a.y - c.y, a.x - c.x) < std::atan2(b.y - c.y, b.x - c.x);
        });

        // 旋转序列，让"最左上"的那个点排第一
        int start = 0;
        float best_s = pts[0].x + pts[0].y;
        for (int k = 1; k < 4; ++k) {
            const float s = pts[k].x + pts[k].y;
            if (s < best_s) { best_s = s; start = k; }
        }
        std::rotate(pts.begin(), pts.begin() + start, pts.end());
        return pts;             
    }

    void publishTfMsg(const std::vector<FtDetecor::armor>& armors, const sensor_msgs::ImageConstPtr &msg, const int& id){
        if(armors.empty()) return;
        // 选重投影误差最小的
        const auto best = std::min_element(armors.begin(), armors.end(),
            [](const FtDetecor::armor& A, const FtDetecor::armor& B) { return A.mean_err < B.mean_err; });

        cv::Mat R;
        cv::Rodrigues(best->rvec, R);
        tf2::Matrix3x3 m(
                    R.at<double>(0,0), R.at<double>(0,1), R.at<double>(0,2),   // 第 0 行
                    R.at<double>(1,0), R.at<double>(1,1), R.at<double>(1,2),   // 第 1 行
                    R.at<double>(2,0), R.at<double>(2,1), R.at<double>(2,2)    // 第 2 行
                );

        tf2::Quaternion q;
        m.getRotation(q);
        q.normalize();

        // 四元数符号连续性（q 与 -q 等价，但会诱发 rviz 突然翻转）
        static tf2::Quaternion q_prev;
        if (q_prev.length2() > 0.5 && q.dot(q_prev) < 0.0)
            q = tf2::Quaternion(-q.x(), -q.y(), -q.z(), -q.w());
        q_prev = q;

        geometry_msgs::TransformStamped ts;
        ts.header            = msg->header;          // 时间戳 + frame_id 一起沿用
        ts.child_frame_id    = "armor_" + std::to_string(id);         // ★ 固定名字，不编号
        ts.transform.translation.x = best->tvec.at<double>(0);
        ts.transform.translation.y = best->tvec.at<double>(1);
        ts.transform.translation.z = best->tvec.at<double>(2);
        ts.transform.rotation      = tf2::toMsg(q);

        tf_broadcaster_.sendTransform(ts);
        

        //画坐标轴
        std::vector<cv::Point3f> axes = {
            {0, 0, 0}, {0.3f, 0, 0}, {0, 0.3f, 0}, {0, 0, 0.3f}
        };
        std::vector<cv::Point2f> proj;
        cv::projectPoints(axes, best->rvec, best->tvec, K_, D_, proj);

        cv::line(debug_, proj[0], proj[1], cv::Scalar(0, 0, 255), 2);   // X 红
        cv::line(debug_, proj[0], proj[2], cv::Scalar(0, 255, 0), 2);   // Y 绿
        cv::line(debug_, proj[0], proj[3], cv::Scalar(255, 0, 0), 2);   // Z 蓝
    }

    void publishDetections(const std::vector<armor>& small,
                       const std::vector<armor>& big,
                       const std_msgs::Header& header)
    {
        ft_detection::DetectionFrame frame;
        frame.header = header;

        uint32_t next_id = 0;
        auto addAll = [&](const std::vector<armor>& list, bool is_large) {
            for (const auto& a : list) {
                ft_detection::Detection d;
                d.id             = next_id++;          // ★ 唯一编号
                d.is_red         = a.is_red;
                d.is_large_armor = is_large;

                d.pose.position.x = a.tvec.at<double>(0);
                d.pose.position.y = a.tvec.at<double>(1);
                d.pose.position.z = a.tvec.at<double>(2);

                cv::Mat R;
                cv::Rodrigues(a.rvec, R);
                tf2::Matrix3x3 m(R.at<double>(0,0), R.at<double>(0,1), R.at<double>(0,2),
                                 R.at<double>(1,0), R.at<double>(1,1), R.at<double>(1,2),
                                 R.at<double>(2,0), R.at<double>(2,1), R.at<double>(2,2));
                tf2::Quaternion q;
                m.getRotation(q);
                q.normalize();
                d.pose.orientation = tf2::toMsg(q);    // ★ 每块板子自己的姿态

                for (int k = 0; k < 4; ++k) {
                    d.corners[k].x = a.points[k].x;
                    d.corners[k].y = a.points[k].y;
                    d.corners[k].z = 0.0f;
                }
                d.confidence = static_cast<float>(1.0 / (1.0 + a.mean_err));

                frame.detections.push_back(d);
            }
        };

        addAll(small, false);
        addAll(big,   true);

        det_pub_.publish(frame);      // ★ 每帧只发一次（空也要发，让下游知道目标消失）
    }



    ros::NodeHandle nh_, pnh_;
    image_transport::ImageTransport it_;
    image_transport::Subscriber image_sub_;
    image_transport::Publisher image_debug_;
    ros::Subscriber camera_info_sub;

    std::string image_topic_, camera_info_topic_, transport_, debug_topic_;
    uint32_t queue_size_;

    sensor_msgs::CameraInfoConstPtr camera_info_;

    //====imageRawCallBack====
    cv::Mat K_,D_;
    cv::Mat debug_;
        //=====装甲板识别=====
    std::vector<HsvRange> red_ranges_, blue_ranges_;
    cv::Mat image_hsv_,mask_;
    cv::Mat mask1_,mask2_;
    //画灯条轮廓
    std::vector<std::vector<cv::Point>> contours_;
    //存放筛选后的灯条
    struct LightBar {
        cv::RotatedRect rect;
        cv::Point2f    center;
        float          length;
        float          width;
        float          angle;
    };

    //=====灯条粗筛阈值（0 = 关闭该项）=====
    double bar_min_area_, bar_max_tilt_;
    double bar_min_long_side_, bar_min_ratio_, bar_max_ratio_, bar_min_fill_;

    //装甲板实际大小，单位：米
    float w_big_;
    float h_big_;
    //物理点
    std::vector<cv::Point3f> object_points_big_;
    tf2_ros::TransformBroadcaster tf_broadcaster_;

    ros::Publisher marker_pub_;
    ros::Publisher det_pub_;
    
    // //放大
    // double zoom_;
    // cv::Mat out_img_;    //没效果，数字大还会卡，待查
};

int main(int argc, char** argv){
    ros::init(argc, argv, "ft_detector");
    FtDetecor node;
    ros::spin();
    return 0;
}