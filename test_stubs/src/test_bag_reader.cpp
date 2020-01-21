#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <string>
#include <iostream>

int frameCount = 0;

void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{

  try
  {
    cv::imshow("view", cv_bridge::toCvShare(msg, "bgr8")->image);
    ROS_INFO("Successfully received image frame.");
    cv::waitKey(10);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
  }

  //write the frames as jpg to a folder (every tenth frame)
  frameCount++;
  std_msgs::Header h = msg->header;

  if (frameCount == 10) {
    ROS_INFO("Saving image");
    std::string timestamp = std::to_string(h.stamp.sec);
    std::string path = "/home/evan/test_images/i_" + timestamp + ".jpeg";
    std::cout << path << std::endl;
    cv::imwrite(path, cv_bridge::toCvShare(msg, "bgr8")->image);
    ROS_INFO("Image saved");
    frameCount = 0;
  }

}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "image_listener");
  ros::NodeHandle nh;
  cv::namedWindow("view");
  cv::startWindowThread();
  image_transport::ImageTransport it(nh);
  image_transport::Subscriber sub = it.subscribe("/ladybug/camera5/image_color", 1, imageCallback);
  ros::spin();
  cv::destroyWindow("view");
}
