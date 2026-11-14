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
        
    }
private:
    void cameraInfoCallBack(const sensor_msgs::CameraInfoConstPtr &msg){
        //ROS_INFO("b_x=%d", msg->binning_x);
        image_info_ = msg;
    }

    void imageRawCallBack(const sensor_msgs::ImageConstPtr &msg){
        if(!image_info_){
            ROS_WARN("null");
            return;
            //应该要用yaml里面的
            /*code*/
        }

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

    sensor_msgs::CameraInfoConstPtr image_info_;
};

int main(int argc, char** argv){
    ros::init(argc, argv, "ft_detector");
    FtDetecor node;
    ros::spin();
    return 0;
}