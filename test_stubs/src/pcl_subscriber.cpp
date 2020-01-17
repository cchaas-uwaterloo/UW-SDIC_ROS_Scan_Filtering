#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/common/common_headers.h>
#include <pcl/features/normal_3d.h>
#include <pcl/io/pcd_io.h>
#include <pcl/console/parse.h>

boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> basic_cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>);
bool visualizer_started = false;

pcl::visualization::PCLVisualizer::Ptr simpleVis (pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud)
{
  pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("Output Points"));
  viewer->setBackgroundColor (0, 0, 0);
  viewer->addPointCloud (cloud, "sample cloud");
  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "sample cloud");
  viewer->addCoordinateSystem (1);
  return (viewer);
}

void visualizer(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud) {
  //visualize the test point cloud
  pcl::visualization::PCLVisualizer::Ptr viewer = simpleVis(cloud);
  viewer->spin();
}

void pclCallback(const sensor_msgs::PointCloud2& pcl_msg) {

  pcl::fromROSMsg(pcl_msg, *basic_cloud_ptr);
  ROS_INFO("Recieved pcl for display.");

  if (!visualizer_started) {
    boost::thread visualizer_thread(visualizer,basic_cloud_ptr);
    visualizer_started = true;
  }

}

int main(int argc, char **argv)
{

  ros::init(argc, argv, "pcl_subscriber");
  ros::NodeHandle nh;
  ros::Subscriber sub = nh.subscribe("/filtered_cloud", 1, pclCallback);
  ros::spin();

  //visualizer_thread_template.join();

}
