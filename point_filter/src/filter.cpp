#include "point_filter/point_filter.h"

namespace point_filter {

PointFilter::PointFilter(ros::NodeHandle& node_handle)
    : nh_(node_handle), nh_priv_("~") {
  if (!ReadParameters()) { ROS_ERROR("Error reading ROS parameters"); }

  //might want to advertise on the public handle of the node (use nh_ rather than nh_priv_)
  publisher_ = nh_priv_.advertise<sensor_msgs::PointCloud2>(publish_topic_, 1);

  publisher_test = nh_priv_.advertise<sensor_msgs::PointCloud2>(publish_topic_test_, 1);

  bool load_intr = LoadIntrinsics();
  bool load_extr = LoadExtrinsics();

  objectFoundCountCam_[0] = 0;
  objectFoundCountCam_[1] = 0;
  objectFoundCountCam_[2] = 0;
  objectFoundCountCam_[3] = 0;
  objectFoundCountCam_[4] = 0;
  objectFoundCountCam_[5] = 0;

  lbLinkName_ = "ladybug_link";
  lbCam0Name_ = "ladybug_cam0";
  lbCam1Name_ = "ladybug_cam1";
  lbCam2Name_ = "ladybug_cam2";
  lbCam3Name_ = "ladybug_cam3";
  lbCam4Name_ = "ladybug_cam4";
  lbCam5Name_ = "ladybug_cam5";
  lidarBaseName_ = "m3d_link";

  reading_camera_ = 0;


  bounding_box_subscriber_ = nh_.subscribe(bounding_box_topic_, 1,
                              &PointFilter::BoundingBoxCallback, this);
  object_count_subscriber_ = nh_.subscribe(object_count_topic_, 1,
                              &PointFilter::ObjectCountCallback, this);
  image_subscriber_ = nh_.subscribe(image_topic_, 1,
                              &PointFilter::ImageCallback, this);
  lidar_subscriber_ = nh_.subscribe(lidar_topic_, 1,
                              &PointFilter::LidarCallback, this);

  /*Test transform*/
  transform_TEST = Eigen::Affine3d::Identity();

  /*Extrinsics transform*/
  //LiDARToCameraTf = extrinsics_.GetTransformEigen('camera','LidAR')

  ROS_INFO("Successfully launched node.");
}

bool PointFilter::ReadParameters() {

  nh_priv_.param<std::string>("bounding_box_topic", bounding_box_topic_,
                              "/darknet_ros/bounding_boxes");
  nh_priv_.param<std::string>("object_count_topic", object_count_topic_,
                              "/darknet_ros/found_object");
  nh_priv_.param<std::string>("publish_topic", publish_topic_,
                              "/test_filtered_cloud");
  nh_priv_.param<std::string>("image_topic", image_topic_,
                              "/F1/image_raw");
  nh_priv_.param<std::string>("lidar_topic", lidar_topic_,
                              "/test_pcl_topic");

  //@TEST
  nh_priv_.param<std::string>("publish_topic_test", publish_topic_test_,
                              "/test_filtered_cloud_inverse");

  ROS_INFO("bounding_box_topic: %s", bounding_box_topic_.c_str());
  ROS_INFO("publish_topic: %s", publish_topic_.c_str());
  ROS_INFO("image_topic: %s", image_topic_.c_str());
  ROS_INFO("lidar_topic: %s", lidar_topic_.c_str());
  return true;
}

bool PointFilter::LoadIntrinsics() {
  std::string file_location = __FILE__;
  file_location.erase(file_location.end() - 28, file_location.end());
  file_location += "/point_filter/config/Intrinsics.json";
  ROS_INFO("Openning Intrinsics: %s", file_location.c_str());
  intrinsics_ = beam_calibration::CameraModel::LoadJSON(file_location);
  return true;
}

bool PointFilter::LoadExtrinsics() {
  std::string file_location = __FILE__;
  file_location.erase(file_location.end() - 28, file_location.end());
  file_location += "/point_filter/config/Extrinsics.json";
  ROS_INFO("Openning Extrinsics: %s", file_location.c_str());
  extrinsics_.LoadJSON(file_location);
  return true;
}

/***********************Callback Functions********************************/

void PointFilter::ImageCallback(const sensor_msgs::Image& image_msg) {
  *imageCam_[reading_camera_] = cv_bridge::toCvCopy(image_msg, sensor_msgs::image_encodings::BGR8)->image;
  reading_camera_++;
  if (reading_camera_ == 6)
    reading_camera_ = 0;
}

void PointFilter::LidarCallback(
    const sensor_msgs::PointCloud2& lidar_msg) {

  //convert ROS message to point cloud type
  PointCloud2::Ptr  scan_tmp_ (new PointCloud2);
  pcl_conversions::toPCL(lidar_msg, *scan_tmp_);
  scan_ = boost::make_shared<PointCloud>();
  filteredPoints_ = boost::make_shared<PointCloud>();
  pcl::fromPCLPointCloud2(*scan_tmp_, *scan_);

  for (int i=camera_t::lbcam0; i < camera_t::lbcam5; i++) {
    //@TEST
    ROS_INFO ("Ran transform loop with camera %d", (int)i -1);

    scan_ = filterObjects(scan_,(camera_t)i);
  }

  pcl::toROSMsg(*scan_, scanOutput_);

  pcl::toROSMsg(*filteredPoints_, filteredOutput_);

  publisher_.publish(scanOutput_);
  publisher_test.publish(filteredOutput_);
}

void PointFilter::BoundingBoxCallback(const darknet_ros_msgs::BoundingBoxesConstPtr& bounding_box_msg) {
  boundingBoxListCam_[reading_camera_] = *bounding_box_msg;
  reading_camera_++;
  if (reading_camera_ == 6)
    reading_camera_ = 0;
}

void PointFilter::ObjectCountCallback(const darknet_ros_msgs::ObjectCountConstPtr& msg) {
  objectFoundCountCam_[reading_camera_] = msg->count;
  reading_camera_++;
  if (reading_camera_ == 6)
    reading_camera_ = 0;
}


/**************************helper functions**********************************/

void PointFilter::transformToCameraFrame(const PointCloud::Ptr input_scan, PointCloud::Ptr out_scan,camera_t camera) {

  Eigen::Affine3d transform;

  if (camera == camera_t::lblink) {
    transform = extrinsics_.GetTransformEigen(lbLinkName_,lidarBaseName_);
  }
  else if (camera == camera_t::lbcam0) {
    transform = extrinsics_.GetTransformEigen(lbCam0Name_,lidarBaseName_);
  }
  else if (camera == camera_t::lbcam1) {
    transform = extrinsics_.GetTransformEigen(lbCam1Name_,lidarBaseName_);
  }
  else if (camera == camera_t::lbcam2) {
    transform = extrinsics_.GetTransformEigen(lbCam2Name_,lidarBaseName_);
  }
  else if (camera == camera_t::lbcam3) {
    transform = extrinsics_.GetTransformEigen(lbCam3Name_,lidarBaseName_);
  }
  else if (camera == camera_t::lbcam4) {
    transform = extrinsics_.GetTransformEigen(lbCam4Name_,lidarBaseName_);
  }
  else if (camera == camera_t::lbcam5) {
    transform = extrinsics_.GetTransformEigen(lbCam5Name_,lidarBaseName_);
  }
  else if (camera == camera_t::lbtest) {
    transform_TEST.translation() << 0, 0, 0;
    transform = transform_TEST;
  }

  pcl::transformPointCloud(*input_scan, *out_scan, transform); //(cloud_in, cloud_out, transform)

}

void PointFilter::transformToLidarFrame(const PointCloud::Ptr input_scan, PointCloud::Ptr out_scan,camera_t camera) {

  Eigen::Affine3d transform;

  if (camera == camera_t::lblink) {
    transform = extrinsics_.GetTransformEigen(lbLinkName_,lidarBaseName_);
  }
  else if (camera == camera_t::lbcam0) {
    transform = extrinsics_.GetTransformEigen(lbCam0Name_,lidarBaseName_);
  }
  else if (camera == camera_t::lbcam1) {
    transform = extrinsics_.GetTransformEigen(lbCam1Name_,lidarBaseName_);
  }
  else if (camera == camera_t::lbcam2) {
    transform = extrinsics_.GetTransformEigen(lbCam2Name_,lidarBaseName_);
  }
  else if (camera == camera_t::lbcam3) {
    transform = extrinsics_.GetTransformEigen(lbCam3Name_,lidarBaseName_);
  }
  else if (camera == camera_t::lbcam4) {
    transform = extrinsics_.GetTransformEigen(lbCam4Name_,lidarBaseName_);
  }
  else if (camera == camera_t::lbcam5) {
    transform = extrinsics_.GetTransformEigen(lbCam5Name_,lidarBaseName_);
  }
  else if (camera == camera_t::lbtest) {
    transform_TEST.translation() << 0, 0, 0;
    transform = transform_TEST;
  }

  transform = transform.inverse();

  pcl::transformPointCloud(*input_scan, *out_scan, transform); //(cloud_in, cloud_out, transform)

}

PointCloud::Ptr PointFilter::filterObjects(PointCloud::Ptr input_scan_, camera_t camera) {

  //set camera variables based on camera param
  uint16_t vmax = imageCam_[(int)camera - 1]->rows;
  uint16_t umax = imageCam_[(int)camera - 1]->cols;
  darknet_ros_msgs::BoundingBoxes bbList = boundingBoxListCam_[(int)camera - 1];
  uint8_t oFCount = objectFoundCountCam_[(int)camera - 1];

  PointCloud::Ptr cameraFrameScan_ = boost::make_shared<PointCloud>();
  PointCloud::Ptr filteredScanCamera_ = boost::make_shared<PointCloud>();
  PointCloud::Ptr filteredScanLidar_ = boost::make_shared<PointCloud>();

  //@TEST
  PointCloud::Ptr filteredCameraPoints_ = boost::make_shared<PointCloud>();

  //make a copy of the input scan in the camera frame
  transformToCameraFrame(input_scan_, cameraFrameScan_, camera);

  transformToCameraFrame(filteredPoints_, filteredCameraPoints_, camera);

  beam::Vec2 coords;
  beam::Vec3 point;
  uint16_t u, v;

  bool behind_plane = false;
  bool beyond_plane = false;

  for (uint32_t i = 0; i < cameraFrameScan_->points.size(); i++) {

    point(0, 0) = cameraFrameScan_->points[i].x;
    point(1, 0) = cameraFrameScan_->points[i].y;
    point(2, 0) = cameraFrameScan_->points[i].z;

    behind_plane = false;
    beyond_plane = false;

    //check that the point is not behind the image plane
    if (point(2,0) < 0)
      behind_plane = true;

    //project the point to the image plane
    coords = intrinsics_->ProjectPoint(point);

    u = std::round(coords(0, 0));
    v = std::round(coords(1, 0));

    //check if the point is beyond the edges of the image
    if (u < 0 || v < 0 || u > umax || v > vmax)
      beyond_plane = true;

    bool filterPoint = false;

    //check if the point is within a bounding box
    for (uint8_t j = 0; j < oFCount; j++) {

      if (u > bbList.bounding_boxes[j].xmin && u < bbList.bounding_boxes[j].xmax &&
          v > bbList.bounding_boxes[j].ymin && v < bbList.bounding_boxes[j].ymax) {
            filterPoint = true;
            ROS_INFO("Filtered point from car %d", j);
          }

    }

      //if the point is not in any bounding boxes, push it to filtered point cloud
      if (!filterPoint || behind_plane || beyond_plane)
        filteredScanCamera_->push_back(cameraFrameScan_->points[i]);
      else
        filteredCameraPoints_->push_back(cameraFrameScan_->points[i]);
  }

  //transform the filtered scan in the camera frame back to the lidar frame
  transformToLidarFrame(filteredScanCamera_, filteredScanLidar_, camera);

  transformToLidarFrame(filteredCameraPoints_, filteredPoints_, camera);

  return (filteredScanLidar_);

}

void PointFilter::testProjection(const PointCloud::Ptr cameraFrameScan_, camera_t camera) {

  beam::Vec2 coords;
  beam::Vec3 point;
  uint16_t u, v;

  for (uint32_t i = 0; i < cameraFrameScan_->points.size(); i++) {
    point(0, 0) = cameraFrameScan_->points[i].x;
    point(1, 0) = cameraFrameScan_->points[i].y;
    point(2, 0) = cameraFrameScan_->points[i].z;

    //check that the point is not behind the image plane
    if (point(2,0) < 0)
      continue;

    //project the point to the image plane
    coords = intrinsics_->ProjectPoint(point);

    u = std::round(coords(0, 0));
    v = std::round(coords(1, 0));

    ROS_INFO ("The projected U coordinate of point %d is %d in camera %d", i, u, camera);
    ROS_INFO ("The projected V coordinate of point %d is %d in camera %d", i, v, camera);


  }

}

} // namespace point_filter
