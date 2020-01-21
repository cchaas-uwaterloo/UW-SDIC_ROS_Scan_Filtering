#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

int main(int argc, char** argv)
{
  std::string files[6] = {
    "/home/evan/test_images/sync_test/c0_100386.jpeg",
    "/home/evan/test_images/sync_test/c1_100386.jpeg",
    "/home/evan/test_images/sync_test/c2_100386.jpeg",
    "/home/evan/test_images/sync_test/c3_100386.jpeg",
    "/home/evan/test_images/sync_test/c4_100386.jpeg",
    "/home/evan/test_images/sync_test/c5_100386.jpeg",
  };

  uint8_t current_image  = 0;

  ros::init(argc, argv, "image_publisher");
  ros::NodeHandle nh;
  image_transport::ImageTransport it(nh);
  image_transport::Publisher pub = it.advertise("camera/rgb/image_raw", 1);

  cv::Mat image[6];
  image[0] = cv::imread(files[0], cv::IMREAD_COLOR);
  image[1] = cv::imread(files[1], cv::IMREAD_COLOR);
  image[2] = cv::imread(files[2], cv::IMREAD_COLOR);
  image[3] = cv::imread(files[3], cv::IMREAD_COLOR);
  image[4] = cv::imread(files[4], cv::IMREAD_COLOR);
  image[5] = cv::imread(files[5], cv::IMREAD_COLOR);

  sensor_msgs::ImagePtr msg[6];
  msg[0] = cv_bridge::CvImage(std_msgs::Header(), "bgr8", image[0]).toImageMsg();
  msg[1] = cv_bridge::CvImage(std_msgs::Header(), "bgr8", image[1]).toImageMsg();
  msg[2] = cv_bridge::CvImage(std_msgs::Header(), "bgr8", image[2]).toImageMsg();
  msg[3] = cv_bridge::CvImage(std_msgs::Header(), "bgr8", image[3]).toImageMsg();
  msg[4] = cv_bridge::CvImage(std_msgs::Header(), "bgr8", image[4]).toImageMsg();
  msg[5] = cv_bridge::CvImage(std_msgs::Header(), "bgr8", image[5]).toImageMsg();

  ros::Rate loop_rate(1);
  while (nh.ok()) {
    pub.publish(msg[current_image]);

    ROS_INFO("Published %d image to camera/rgb/image_raw topic", current_image);

    current_image++;

    if (current_image==6)
      current_image = 0;

    ros::spinOnce();
    loop_rate.sleep();
  }
}
