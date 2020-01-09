#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "image_publisher");
  ros::NodeHandle nh;
  image_transport::ImageTransport it(nh);
  image_transport::Publisher pub = it.advertise("camera/rgb/image_raw", 1);

  cv::Mat image = cv::imread(argv[1], cv::IMREAD_COLOR);

  if (image.cols == 0 && image.rows == 0)
  {
    ROS_ERROR ("Empty image loaded to publisher - specify image path as cmd arg");
  }

  sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", image).toImageMsg();

  ros::Rate loop_rate(0.1);
  while (nh.ok()) {
    pub.publish(msg);

    ROS_INFO("Published image to camera/rgb/image_raw topic");

    ros::spinOnce();
    loop_rate.sleep();
  }
}
