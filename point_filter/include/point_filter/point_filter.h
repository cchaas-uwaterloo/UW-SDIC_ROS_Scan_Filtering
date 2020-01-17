#pragma once

#include "beam_calibration/CameraModel.h"
#include "beam_calibration/PinholeCamera.h"
#include "beam_calibration/TfTree.h"
#include <darknet_ros_msgs/BoundingBox.h>
#include <geometry_msgs/Twist.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/transforms.h>
#include <cv_bridge/cv_bridge.h>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_srvs/Trigger.h>
#include <darknet_ros_msgs/BoundingBoxes.h>
#include <darknet_ros_msgs/BoundingBox.h>
#include <darknet_ros_msgs/ObjectCount.h>
#include "beam_utils/math.hpp"

typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloud;
typedef pcl::PCLPointCloud2 PointCloud2;

enum camera_t {
  lblink,
  lbcam0,
  lbcam1,
  lbcam2,
  lbcam3,
  lbcam4,
  lbcam5,
  lbtest,
  lbLast
};

namespace point_filter {

class PointFilter {
public:
  explicit PointFilter(ros::NodeHandle& node_handle);

  ~PointFilter() = default;

private:
  /**
   * Function for reading ROS parameters on initialization.
   * @return Returns true if successful.
   */
  bool ReadParameters();

  bool LoadIntrinsics();

  bool LoadExtrinsics();

  /*
  * @brief callback function for image topic subscription
  */
  void ImageCallback(const sensor_msgs::Image& image_msg);

  /*
  * @brief callback function for Lidar topic subscription
  */
  void LidarCallback(const sensor_msgs::PointCloud2& lidar_msg);

  /*
  * @brief callback funtion for Bounding Box topic subscription
  */
  void BoundingBoxCallback(const darknet_ros_msgs::BoundingBoxesConstPtr& bounding_box_msg);

  /*
  * @brief callback funcion for found_object count topic subscription
  * @descr filtering and publishing to filtered topic done here
  */
  void ObjectCountCallback(const darknet_ros_msgs::ObjectCountConstPtr& msg);

  ros::NodeHandle& nh_;      // ROS node handle.
  ros::NodeHandle nh_priv_;  // Private ROS node handle.
  ros::Publisher publisher_; // ROS publisher for filtered cloud
  beam_calibration::TfTree extrinsics_; // Robot extrinsics (pysical calibrations between sensors)
  std::shared_ptr<beam_calibration::CameraModel>
      intrinsics_; // camera intrinsics (internal camera calibrations)
  ros::Subscriber image_subscriber_, lidar_subscriber_,
      bounding_box_subscriber_, object_count_subscriber_;
  std::string bounding_box_topic_, image_topic_, lidar_topic_, publish_topic_, object_count_topic_;

  PointCloud::Ptr scan_; //raw point cloud comming from LiDAR topic
  PointCloud::Ptr cameraFrameScan_; //point cloud transformed to camera frame


  PointCloud::Ptr filteredScan_; //filtered point cloud with vehicles and people removed
  sensor_msgs::PointCloud2 filteredScanOutput_; //filtered output converted to ROS message format

  cv::Mat *imageCam0_;
  cv::Mat *imageCam1_;
  cv::Mat *imageCam2_;
  cv::Mat *imageCam3_;
  cv::Mat *imageCam4_;
  cv::Mat *imageCam5_;

  darknet_ros_msgs::BoundingBoxesConstPtr boundingBoxListCam0;
  darknet_ros_msgs::BoundingBoxesConstPtr boundingBoxListCam1;
  darknet_ros_msgs::BoundingBoxesConstPtr boundingBoxListCam2;
  darknet_ros_msgs::BoundingBoxesConstPtr boundingBoxListCam3;
  darknet_ros_msgs::BoundingBoxesConstPtr boundingBoxListCam4;
  darknet_ros_msgs::BoundingBoxesConstPtr boundingBoxListCam5;

  uint8_t  objectFoundCountCam0_;
  uint8_t  objectFoundCountCam1_;
  uint8_t  objectFoundCountCam2_;
  uint8_t  objectFoundCountCam3_;
  uint8_t  objectFoundCountCam4_;
  uint8_t  objectFoundCountCam5_;

  std::string lbLinkName_, lbCam0Name_, lbCam1Name_, lbCam2Name_, lbCam3Name_,
          lbCam4Name_, lbCam5Name_, lidarBaseName_;

  /*test transform*/
  Eigen::Affine3d transform_TEST;

  /*
  * @brief Transforms the raw point cloud in LiDAR reference frame to camers's reference frame
  * @params Takes raw point cloud data received via ROS Topic and enumerated camera to transform to
  * @return Returns tranformed point cloud in camera's reference frame
  */
  PointCloud::Ptr transformToCameraFrame(const PointCloud::Ptr input_scan_, camera_t camera);

  /*
  * @brief Filters out point cloud points identified as belonging to vehicles or other filterObjects
  * @params Takes point cloud data in camera's reference frame
  * @return filtered point cloud
  */
  void filterObjects(const PointCloud::Ptr input_scan_, camera_t camera);

  /*
  * @Test
  * @brief Tests projection of points to camera frame(s) and outputs coordinates
  * @params Takes point cloud in camera's reference frame
  * @Return void
  */
  void testProjection(const PointCloud::Ptr cameraFrameScan_, camera_t camera);

};

} // namespace beam_follower
