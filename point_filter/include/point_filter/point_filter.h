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
#include <std_msgs/Int16.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/common/common_headers.h>
#include <pcl/features/normal_3d.h>
#include <pcl/io/pcd_io.h>
#include <pcl/console/parse.h>
#include <opencv2/datasets/gr_skig.hpp>
#include <beam_calibration/LadybugCamera.h>

typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloud;
typedef pcl::PCLPointCloud2 PointCloud2;

#define NUM_CAMERAS 6
#define CAMERA_INDEX(camera) (int)camera-1

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


  //**************************************************************************//
  //******************************SETUP FUNCTIONS*****************************//
  //**************************************************************************//


  /*
   * @brief Function for reading ROS parameters on initialization.
   * @return Returns true if successful.
   */
  bool ReadParameters();

  /*
   * @brief Function to load intrinsic calibrations for all ladybug camera
   * @return Returns true if successful
   */
  bool LoadIntrinsics();

  /*
   * @brief Function to load extrinsic calibrations between sensors
   * @return Returns true if successful
   */
  bool LoadExtrinsics();


  //**************************************************************************//
  //****************************CALLBACK FUNCTIONS****************************//
  //**************************************************************************//


  /*
   * @brief Callback funcion for darknet image topic subscription
   * @description Reads in darknet images with bounding box overlays to the current
                 reading camera
   * @params
   *   image_msg: darknet image with bounding box overlays
   * @return void
   */
  void ImageCallback(const sensor_msgs::Image& image_msg);

  /*
   * @brief callback funcion for LiDAR scan topic subscription
   * @description Reads in point cloud from the LiDAR topic and calls the filter
                 function with all 6 cameras. Publishes the filtered cloud and
                 inverted filtered cloud once filtering is complete
   * @params
   *   lidar_msg: raw point cloud provided by LiDAR scanner
   * @return void
   */
  void LidarCallback(const sensor_msgs::PointCloud2& lidar_msg);

  /*
   * @brief callback funcion for bounding box list topic subscription
   * @description Reads the bounding box information to the current reading camera
   *              if darknet has published a new info set. If all the required info
                 for that image has been read, publishes the acknowledgement.
   * @params
   *   bounding_box_msg: list of bounding boxes in the current image
   * @return void
   */
  void BoundingBoxCallback(const darknet_ros_msgs::BoundingBoxesConstPtr& bounding_box_msg);

  /*
   * @brief callback funcion for found object count topic subscription
   * @description Reads the object count information to the current reading camera
   *              if darknet has published a new info set. If all the required info
                 for that image has been read, publishes the acknowledgement.
   * @params
   *   msg: number of objects detected in the current image
   * @return void
   */
  void ObjectCountCallback(const darknet_ros_msgs::ObjectCountConstPtr& msg);

  /*
   * @brief Callback funtion for Camera Number topic subscription
   * @description Reads in and updates the camera number the incoming data is
                 is for
   * @params
   *   msg number of the camera for which data will be provided from darknet
   * @return void
   */
  void CameraNumberCallback(const std_msgs::Int16 msg);


  //**************************************************************************//
  //******************************MEMBER VARIABLES****************************//
  //**************************************************************************//


  ros::NodeHandle& nh_;                                                         // ROS node handle.
  ros::NodeHandle nh_priv_;                                                     // Private ROS node handle.
  ros::Publisher publisher_, publisher_original_, publisher_ack_,               // ROS publishers
                 publisher_pclack_;
  ros::Subscriber image_subscriber_, lidar_subscriber_,                         // ROS subscribers
                  bounding_box_subscriber_, object_count_subscriber_,
                  camera_number_subscriber_;
  std::string bounding_box_topic_, image_topic_, lidar_topic_,                  // ROS topic names
              publish_topic_, object_count_topic_, publish_original_topic_,
              camera_number_topic_, publish_ack_topic_, publish_pclack_topic_;

  beam_calibration::TfTree extrinsics_;                                         // Robot extrinsics (pysical calibrations between sensors)
  std::shared_ptr<beam_calibration::CameraModel> intrinsics_[NUM_CAMERAS];      // Camera intrinsics (internal camera calibrations)

  PointCloud::Ptr scan_;                                                        // Raw point cloud comming from LiDAR topic, filtered successively by image
  sensor_msgs::PointCloud2 scanOutput_;                                         // Filtered scan cloud converted to ROS message format

  std_msgs::Int16 ack_m;                                                        // Message used to acknowlege receipt of image once all image data received
  cv::Mat imageCam_[NUM_CAMERAS];                                               // Current images from each camera
  cv::Mat lastImage_;                                                           // Last image read in from darknet, used to identify when new image is provided
  uint8_t imageSem_;                                                            // Semaphore blocking acknowledgement of current image until object count and bb data recieved
  uint8_t reading_camera_;                                                      // Camera to which data being read from darknet corresponds
  bool acknowledgeSent_;                                                         // Latch variable to only send acknowledge once each time a new set of image data is recieved

  darknet_ros_msgs::BoundingBoxes boundingBoxListCam_[NUM_CAMERAS];             // Current bounding box list from each camera
  uint8_t objectFoundCountCam_[NUM_CAMERAS];                                    // Current object list from each each camera

  float filterIntensity_;                                                       // Factor affecting how much gets filtered, a higher value corresponds to more being filtered from the point scan


  std::string lbLinkName_, lbCam0Name_, lbCam1Name_, lbCam2Name_, lbCam3Name_,  // Coordinate frame names corresponding to the tfTree transforms
              lbCam4Name_, lbCam5Name_, lidarBaseName_;

  uint8_t firstImages_;                                                         // Used to discard the first two images provided by darknet on initialization


  //**************************************************************************//
  //*****************************UTILITY FUNCTIONS****************************//
  //**************************************************************************//


  /*
   * @brief Transforms the point cloud in the LiDAR reference frame to the camera's reference frame
   * @description Uses extrinsic calibration transforms read in from Extrinsics.json to perform
                 transformations between LiDAR frame and specified camera frame
   * @params
   *   input_scan: point cloud in LiDAR frame
   *   out_scan: point cloud in camera frame
   *   camera: camera of out scan frame
   * @return void
   */
  void transformToCameraFrame(const PointCloud::Ptr input_scan, PointCloud::Ptr out_scan, camera_t camera);

  /*
   * @brief Transforms the point cloud in the camera's reference frame to the LiDAR reference frame
   * @description Uses extrinsic calibration transforms read in from Extrinsics.json to perform
                 transformations between specified camera frame and LiDAR frame
   * @params
   *   input_scan: point cloud in camera frame
   *   input_scan: point cloud in LiDAR frame
   *   camera: camera of input_scan frame
   * @return void
   */
  void transformToLidarFrame(const PointCloud::Ptr input_scan, PointCloud::Ptr out_scan, camera_t camera);

  /*
   * @brief Filters out point cloud points identified as belonging to vehicles or other filterObjects
   * @description Transforms the input_scan point cloud to the specified camera frame and projects
                 each point in the cloud to the camera plane. If the point does not project to within
                 a bounding box on the image plane, it is written back to the filtered cloud. Points
                 projecting to within a bounding box are not written back to the filtered cloud unless
                 they are identified by the restoreOutliers function as not belonging to the filtered
                 object. The filtered cloud is then restored to the LiDAR frame.
   * @params
   *   input_scan raw input point cloud in LiDAR frame
   *   camera camera image on which cloud is being filtered
   * @return filtered point cloud
   */
  PointCloud::Ptr filterObjects(PointCloud::Ptr input_scan, camera_t camera);

  /*
   * @brief Restores filtered background points to the filtered cloud
   * @description Calculates the mean of all the z-values of the points in each bounding box and restores those that are
                 more than a specified number of standard deviations from the mean to the filtered cloud
   * @params
   *   cloud: filtered point cloud to which points will be restored
   *   objectCount: number of objects found in the current camera image
   *   bbPoints: each point identified in each bounding box, bounding boxes > points
   *   factor: multiplier used to control how many points are restored, it is the number of standard deviations from
              the mean of all the z-values of the points in a bouding box that a point's z-value must be for it to be
              restored
   * @return void
   */
  void restoreOutliers(PointCloud::Ptr& cloud, uint8_t objectCount, const std::vector<std::vector<pcl::PointXYZ>>& bbPoints, float factor);

  /*
   * @brief Helper function to compare two images
   * @description Performs image subtraction on the provided Mats to determine if they are equal, returns false
                 if the images are different sizes and true if the images are empty
   * @params
   *   mat1: first image to compare
   *   mat2: second image to compare
   * @return true if the images are equal, false if the images are different
   */
  bool imageIsEqual(const cv::Mat mat1, const cv::Mat mat2);


  //**************************************************************************//
  //******************************DEBUG FUNCTIONS*****************************//
  //**************************************************************************//


  /*
   * @TEST
   * @brief Tests projection of points to camera frame(s) and outputs coordinates
   * @params Takes point cloud in camera's reference frame
   * @Return void
   */
  void testProjection(const PointCloud::Ptr cameraFrameScan_, camera_t camera);

  /*
   * @TEST
   * @brief Test function to visualize point clouds
   * @note Not multi-threaded so will halt program execution until closed
   */
  void visualizer(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, std::string displayName);

  /*
   * @TEST
   * @brief saves the camera image overlayed with the projected points and bounding boxes
   */
  void displayProjectionImage(camera_t camera, const cv::Mat image, std::vector<beam::Vec2> projectedPoints);



};

} // namespace beam_follower
