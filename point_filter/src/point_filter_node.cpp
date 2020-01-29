#include "point_filter/point_filter.h"
#include <ros/ros.h>
#include <ros/subscriber.h>

int main(int argc, char** argv) {
  ros::init(argc, argv, "point_filter");
  ros::NodeHandle node_handle("~");

  ros::AsyncSpinner spinner(0);
  spinner.start();

  point_filter::PointFilter point_filter_node{node_handle};

  ros::waitForShutdown();
  //ros::spin();
  return 0;
}
