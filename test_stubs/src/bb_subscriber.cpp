#include <ros/ros.h>
//#include <image_transport/image_transport.h>
#include <darknet_ros_msgs/BoundingBoxes.h>
#include <darknet_ros_msgs/BoundingBox.h>
#include <darknet_ros_msgs/ObjectCount.h>
//#include <opencv2/highgui/highgui.hpp>
//#include <cv_bridge/cv_bridge.h>

int objectCountVar = 0;

class BoundingBoxClass {
public:

  darknet_ros_msgs::BoundingBoxesConstPtr memberMsg;
  int countMember;
  bool read_once;

  BoundingBoxClass(void)
  {
    //memberMsg = NULL;
    countMember = 0;
    read_once = false;
  }

  ~BoundingBoxClass(void)
  {
    //delete memberMsg;
  }

  void boundingBoxCallBack(const darknet_ros_msgs::BoundingBoxesConstPtr& msg)
  {
    ROS_INFO ("BoundingBox data received.");
    memberMsg = msg;
    read_once = true;

    for (int i = 0; i < countMember; i++)
    {
      ROS_INFO ("BoundingBox %d dimensions:", i);
      ROS_INFO ("xmin: %d", msg->bounding_boxes[i].xmin);
      ROS_INFO ("xmax: %d", msg->bounding_boxes[i].xmax);
      ROS_INFO ("ymin: %d", msg->bounding_boxes[i].ymin);
      ROS_INFO ("ymax: %d", msg->bounding_boxes[i].ymax);
    }

  }

  void objectsCallBack(const darknet_ros_msgs::ObjectCountConstPtr& msg)
  {
    ROS_INFO ("%d objects detected.", msg->count);
    countMember = msg->count;

  }

  darknet_ros_msgs::BoundingBoxesConstPtr getMessage()
  {
    return memberMsg;
  }

  int getCount()
  {
    return countMember;
  }

};

int main(int argc, char **argv)
{

  BoundingBoxClass bbInterface;

  ros::init(argc, argv, "bounding_box_listener");
  ros::NodeHandle nh;
  ros::Subscriber sub1 = nh.subscribe("darknet_ros/found_object", 1, &BoundingBoxClass::objectsCallBack, &bbInterface);
  ros::Subscriber sub2 = nh.subscribe("/darknet_ros/bounding_boxes", 1, &BoundingBoxClass::boundingBoxCallBack, &bbInterface);

  ROS_INFO ("%d", bbInterface.read_once);

  darknet_ros_msgs::BoundingBoxesConstPtr msg = bbInterface.getMessage();
  int count = bbInterface.getCount();

  if (bbInterface.read_once == true) {
    for (int i = 0; i < count; i++)
    {
      ROS_INFO ("BoundingBox %d dimensions:", i);
      ROS_INFO ("xmin: %d", msg->bounding_boxes[i].xmin);
      ROS_INFO ("xmax: %d", msg->bounding_boxes[i].xmax);
      ROS_INFO ("ymin: %d", msg->bounding_boxes[i].ymin);
      ROS_INFO ("ymax: %d", msg->bounding_boxes[i].ymax);
    }

  }

  ros::spin();

}
