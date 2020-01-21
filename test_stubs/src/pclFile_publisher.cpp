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
#include <boost/thread/thread.hpp>
#include <stdlib.h>

ros::Publisher pub;


pcl::visualization::PCLVisualizer::Ptr simpleVis (pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud)
{
  pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("Input Points"));
  viewer->setBackgroundColor (0, 0, 0);
  viewer->addPointCloud (cloud, "sample cloud");
  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud");
  viewer->addCoordinateSystem (1);
  return (viewer);
}

void visualizer(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud) {
  //visualize the test point cloud
  pcl::visualization::PCLVisualizer::Ptr viewer = simpleVis(cloud);
  viewer->spin();
}


main (int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "pcl_publisher");
  ros::NodeHandle nh;

  // Create a ROS publisher for the output point cloud
  pub = nh.advertise<sensor_msgs::PointCloud2> ("/test_pcl_topic", 1);

  //create point cloud with one point and convert to ROS message format
  boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> basic_cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>);

  if (pcl::io::loadPCDFile ("/home/evan/test_scans/s_100386.pcd", *basic_cloud_ptr) == -1)
  {
     ROS_INFO("could not load the point cloud");
     return -1;
  }
  ROS_INFO("Successfully loaded point cloud");

  sensor_msgs::PointCloud2 output;
  pcl::toROSMsg(*basic_cloud_ptr, output);

  //@TEST
  ROS_INFO("successfully converted to ros message type");

  //start a second thread for the visualizer
  boost::thread visualizer_thread(visualizer,basic_cloud_ptr);
  ROS_INFO("Successfully started visualizer thread");


  // Spin and publish
  ros::Rate loop_rate(0.1);
  while (nh.ok()) {

    ROS_INFO("Publishing PCL");

    pub.publish(output);

    //viewer->spinOnce(100);
    ros::spinOnce();
    loop_rate.sleep();
  }

  visualizer_thread.join();

}
