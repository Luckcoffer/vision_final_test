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

class FtDetecor{
public:
    FtDetecor() : nh_(), pnh_("~"), it_(nh_){
        image_topic_ = pnh_.param<std::string>("image_topic","/hk_camera/image_raw/");
        camera_info_topic_ = pnh_.param<std::string>("camera_info_topic","/hk_camera/camera_info");
        transport_ = pnh_.param<std::string>("transport", "compressed");
        queue_size_ = pnh_.param<int>("queue_size", 2);
        debug_topic_ = pnh_.param<std::string>("debug_topic", "/ft/debug/image");

        w_big_ = pnh_.param<float>("w_big", 0.235f);
        h_big_ = pnh_.param<float>("l_big", 0.127f);

        object_points_big_ = {
            {-w_big_/2,  h_big_/2, 0.f},     //TL
            { w_big_/2,  h_big_/2, 0.f},     //TR
            { w_big_/2, -h_big_/2,  0.f},    //BR
            {-w_big_/2, -h_big_/2,  0.f}     //BL
        };

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
        cv::inRange(image_hsv_,cv::Scalar(0, 100, 200),cv::Scalar(34, 255, 255),mask1_);
        cv::inRange(image_hsv_,cv::Scalar(170, 100, 200),cv::Scalar(179, 255, 255),mask2_);
        cv::bitwise_or(mask1_, mask2_, mask_);

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
            if(area < 10.0) continue;

            //长宽高过滤，有比值，筛出细长的
            // const cv::Rect r = cv::boundingRect(contour);
            // if(r.width <= 0) continue;
            // const double ratio = static_cast<double>(r.height) / r.width;
            // if(r.height < 20 || ratio < 2.0) continue;
            //旋转矩形（不受倾斜影响）
            const cv::RotatedRect rr = cv::minAreaRect(contour);
            const float long_side = std::max(rr.size.width, rr.size.height);
            const float short_side = std::min(rr.size.height, rr.size.width);
            //if(short_side < 0.0f) continue;

            const double ratio = static_cast<double>(long_side / short_side);
            //if(long_side < 10.0f || ratio < 2.0 || ratio > 12.0) continue;

            // 填充率：排除 L 形、散点、锯齿 轮廓面积/矩形面积
            const double fill = area / static_cast<double>(long_side * short_side);
            //if (fill < 0.5) continue;

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
            if (tilt > 40.0) continue;         // 灯条不会斜过 40°

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
        std::vector<std::array<cv::Point2f, 4>> armors;   // 每块装甲板的 4 角点（TL,TR,BR,BL）,定义1
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
                ROS_INFO("1");

                cv::Point2f pa[4],pb[4];
                a.rect.points(pa);
                b.rect.points(pb);

                std::vector<cv::Point2f> all(pa,pa+4);
                all.insert(all.end(), pb, pb + 4);

                std::vector<cv::Point2f> hull;
                cv::convexHull(all,hull);
                if(hull.size() < 4) continue;

                ROS_INFO("2");

                // ---------- 6) 凸包 → 4 边形 ----------
                std::vector<cv::Point2f> quad;
                const double eps = 0.03 * cv::arcLength(hull, true);
                cv::approxPolyDP(hull, quad, eps, true);
                if (quad.size() != 4) continue;

                ROS_INFO("3");

                // ---------- 7) 排序成 TL, TR, BR, BL ----------
                const auto ordered = orderPointsByAngle(quad);
                if (ordered.size() != 4) continue;

                ROS_INFO("4");

                // ---------- 8) 形状校验 ----------
                const float w = static_cast<float>(cv::norm(ordered[1] - ordered[0]));  // 上边长
                const float h = static_cast<float>(cv::norm(ordered[3] - ordered[0]));  // 左边长
                if (w < 1.0f || h < 1.0f) continue;

                const double ar = static_cast<double>(h) / w;      // 装甲板大致 1:1 ~ 1:2
                if (ar < 0.0 || ar > 2.5) continue;

                ROS_INFO("5");

                if (cv::contourArea(ordered) < 300.0) continue;


                // ---------- 9) 记录 + 画红框 ----------
                armors.push_back({ordered[0], ordered[1], ordered[2], ordered[3]});

                for (int k = 0; k < 4; ++k) {
                cv::line(debug_, ordered[k], ordered[(k + 1) % 4],
                     cv::Scalar(0, 0, 255), 2);            // 红框
                }
                const cv::Point2f ctr = (ordered[0] + ordered[1] + ordered[2] + ordered[3]) * 0.25f;
                cv::circle(debug_, ctr, 3, cv::Scalar(0, 255, 255), -1);   // 黄色中心点

                //====位姿解算与TF====
                cv::Mat rvec, tvec;
                bool ok = cv::solvePnP(object_points_big_, ordered, K_, D_, rvec, tvec, false, cv::SOLVEPNP_IPPE);
                if(!ok){
                    ROS_WARN("solvePnP failed");
                    continue;
                }

                //旋转矩阵
                cv::Mat R;  
                cv::Rodrigues(rvec, R);

                //四元数
                tf2::Matrix3x3 m(
                    R.at<double>(0,0), R.at<double>(0,1), R.at<double>(0,2),   // 第 0 行
                    R.at<double>(1,0), R.at<double>(1,1), R.at<double>(1,2),   // 第 1 行
                    R.at<double>(2,0), R.at<double>(2,1), R.at<double>(2,2)    // 第 2 行
                );

                tf2::Quaternion q;
                m.getRotation(q);
                q.normalize();

                geometry_msgs::TransformStamped ts;
                ts.header.stamp = msg->header.stamp;
                ts.header.frame_id = msg->header.frame_id;
                ts.child_frame_id = "arrmor_";
                ++id;

                ts.transform.translation.x = tvec.at<double>(0);
                ts.transform.translation.y = tvec.at<double>(1);
                ts.transform.translation.z = tvec.at<double>(2);
                ts.transform.rotation      = tf2::toMsg(q);

                tf_broadcaster_.sendTransform(ts);

                // double x = tvec.at<double>(0);
                // double y = tvec.at<double>(1);
                // double z = tvec.at<double>(2);
                // double distance = cv::norm(tvec);

                //验证
                std::vector<cv::Point2f> reproj;
                cv::projectPoints(object_points_big_, rvec, tvec, K_, D_, reproj);
                double sum = 0, max_err = 0;
                for (size_t i = 0; i < reproj.size(); ++i) {
                    double e = cv::norm(reproj[i] - ordered[i]);
                    sum += e;
                    max_err = std::max(max_err, e);
                    cv::line(debug_, reproj[i], reproj[(i + 1) % 4],
                                cv::Scalar(255, 0, 255), 2);    
                }        
                double mean_err = sum / reproj.size();
                ROS_INFO("reproj: mean=%.2f px, max=%.2f px", mean_err, max_err);

                //画坐标轴
                std::vector<cv::Point3f> axes = {
                    {0, 0, 0}, {0.3f, 0, 0}, {0, 0.3f, 0}, {0, 0, 0.3f}
                };
                std::vector<cv::Point2f> proj;
                cv::projectPoints(axes, rvec, tvec, K_, D_, proj);

                cv::line(debug_, proj[0], proj[1], cv::Scalar(0, 0, 255), 2);   // X 红
                cv::line(debug_, proj[0], proj[2], cv::Scalar(0, 255, 0), 2);   // Y 绿
                cv::line(debug_, proj[0], proj[3], cv::Scalar(255, 0, 0), 2);   // Z 蓝
            }
        }

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

    //返回配对成功的两个灯条的外角点



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

    //装甲板实际大小，单位：米
    float w_big_;
    float h_big_;
    //物理点
    std::vector<cv::Point3f> object_points_big_;
    tf2_ros::TransformBroadcaster tf_broadcaster_;
    
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