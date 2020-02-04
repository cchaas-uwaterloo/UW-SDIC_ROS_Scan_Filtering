#include "point_filter/point_filter.h"
#include <ros/ros.h>
#include <ros/subscriber.h>

int main(int argc, char** argv) {
  ros::init(argc, argv, "point_filter");
  ros::NodeHandle node_handle("~");

  /*
   * @INFO Asynchronous spinner used to spin each sub callback in its own thread.
   *       This is mainly required to keep receiving messages on the image
   *       and bounding box topics while the pcl is being filtered in the
   *       lidar callback function.
   */

  ros::AsyncSpinner spinner(0);
  spinner.start();

  point_filter::PointFilter point_filter_node{node_handle};

  ros::waitForShutdown();
  return 0;
}
