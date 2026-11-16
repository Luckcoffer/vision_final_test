#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/image_encodings.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>        //ROS和cv信息的桥梁
#include <opencv2/opencv.hpp>

class FtDetecor{
public:
    FtDetecor() : nh_(), pnh_("~"), it_(nh_){
        image_topic_ = pnh_.param<std::string>("image_topic","/hk_camera/image_raw/");
        camera_info_topic_ = pnh_.param<std::string>("camera_info_topic","/hk_camera/camera_info");
        transport_ = pnh_.param<std::string>("transport", "compressed");
        queue_size_ = pnh_.param<int>("queue_size", 2);
        debug_topic_ = pnh_.param<std::string>("debug_topic", "/ft/debug/image");

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

        cv::Mat debug = bgr.clone();
        //装甲板识别


        cv_bridge::CvImage out(msg->header,sensor_msgs::image_encodings::BGR8,debug);
        image_debug_.publish(out.toImageMsg());

        // test
        // cv::imshow("1",bgr);
        // cv::waitKey(1);
    }

    ros::NodeHandle nh_, pnh_;
    image_transport::ImageTransport it_;
    image_transport::Subscriber image_sub_;
    image_transport::Publisher image_debug_;
    ros::Subscriber camera_info_sub;

    std::string image_topic_, camera_info_topic_, transport_, debug_topic_;
    uint32_t queue_size_;

    sensor_msgs::CameraInfoConstPtr camera_info_;
    cv::Mat K_,D_;
};

int main(int argc, char** argv){
    ros::init(argc, argv, "ft_detector");
    FtDetecor node;
    ros::spin();
    return 0;
}