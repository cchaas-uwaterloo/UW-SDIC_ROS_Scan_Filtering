#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/CompressedImage.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

#include <boost/foreach.hpp>
#define foreach BOOST_FOREACH

/*
void imageCallback(const sensor_msgs::ImageConstPtr & msg)
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
}
*/

int main(int argc, char** argv)
{

  ROS_INFO("Placeholder");
  //rosbag::Bag bag;
  //bag.open("~/Jan14th_UWGarageBag_V01.bag", rosbag::bagmode::Read);
  /*
  std::vector<std::string> topic;
  topic.push_back(std::string("/ladybug/camera0/image_color"));
  //topics.push_back(std::string("/ladybug/camera1/image_color/compressed"));
  //topics.push_back(std::string("/ladybug/camera2/image_color/compressed"));
  //topics.push_back(std::string("/ladybug/camera3/image_color/compressed"));
  //topics.push_back(std::string("/ladybug/camera4/image_color/compressed"));
  //topics.push_back(std::string("/ladybug/camera5/image_color/compressed"));



  rosbag::View view(bag, rosbag::TopicQuery(topic));

  int count = 0;

  foreach(rosbag::MessageInstance const m, view)
  {
      sensor_msgs::ImageConstPtr i = m.instantiate<sensor_msgs::CompressedImage>();
      if (i != NULL)
          imageCallback(i);

      count++;

      if (count > 100)
        exit;

  }
  bag.close();
  */
}
