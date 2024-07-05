#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <opencv2/highgui/highgui.hpp>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "image_publisher");
    ros::NodeHandle nh;
    ros::NodeHandle nh_private_("~");
    image_transport::ImageTransport it(nh);
    int camera_id=0;
    image_transport::Publisher pub = it.advertise("camera/image", 10);
     nh_private_.getParam("camera_id", camera_id);
    cv::VideoCapture cap(camera_id);
    if(!cap.isOpened()) {
        ROS_ERROR("Cannot open video capture");
        return -1;
    }

    cv::Mat frame;
    sensor_msgs::ImagePtr msg;

    ros::Rate loop_rate(5);
    while (nh.ok()) {
        cap >> frame;
        if(!frame.empty()) {
            msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", frame).toImageMsg();
            pub.publish(msg);
            cv::waitKey(1);
        }
    }
}
