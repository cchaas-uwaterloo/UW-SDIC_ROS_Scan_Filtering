#include "point_filter/point_filter.h"

namespace point_filter {

PointFilter::PointFilter(ros::NodeHandle& node_handle)
    : nh_(node_handle), nh_priv_("~") {
  if (!ReadParameters()) { ROS_ERROR("Error reading ROS parameters"); }

  //might want to advertise on the public handle of the node (use nh_ rather than nh_priv_)
  publisher_ = nh_priv_.advertise<sensor_msgs::PointCloud2>(publish_topic_, 1);

  bool load_intr = LoadIntrinsics();
  bool load_extr = LoadExtrinsics();

  objectFoundCountCam0_ = 0;
  objectFoundCountCam1_ = 0;
  objectFoundCountCam2_ = 0;
  objectFoundCountCam3_ = 0;
  objectFoundCountCam4_ = 0;
  objectFoundCountCam5_ = 0;

  lbLinkName_ = "ladybug_link";
  lbCam0Name_ = "ladybug_cam0";
  lbCam1Name_ = "ladybug_cam1";
  lbCam2Name_ = "ladybug_cam2";
  lbCam3Name_ = "ladybug_cam3";
  lbCam4Name_ = "ladybug_cam4";
  lbCam5Name_ = "ladybug_cam5";
  lidarBaseName_ = "m3d_link";

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
                              "/bounding_box");
  nh_priv_.param<std::string>("publish_topic", publish_topic_,
                              "/filtered_cloud");
  nh_priv_.param<std::string>("image_topic", image_topic_,
                              "/F1/image_raw");
  nh_priv_.param<std::string>("lidar_topic", lidar_topic_,
                              "/test_pcl_topic");

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
  *imageCam0_ = cv_bridge::toCvCopy(image_msg, sensor_msgs::image_encodings::BGR8)->image;
}

void PointFilter::LidarCallback(
    const sensor_msgs::PointCloud2& lidar_msg) {

  //@TEST
  ROS_INFO("Reading pcl.");

  //convert ROS message to point cloud type
  PointCloud2::Ptr  scan_tmp_ (new PointCloud2);
  pcl_conversions::toPCL(lidar_msg, *scan_tmp_);
  scan_ = boost::make_shared<PointCloud>();
  pcl::fromPCLPointCloud2(*scan_tmp_, *scan_);

  //@TEST
  ROS_INFO("successfully converted pcl ROS message to point cloud");

  //Transform raw lidar scan to all camera frames and filter in each frame
  cameraFrameScan_ = boost::make_shared<PointCloud>();

  for (int i=camera_t::lblink; i < camera_t::lbcam0; i++) {

    //@TEST
    ROS_INFO ("Ran transform loop with camera %d", (int)i);
    cameraFrameScan_ = transformToCameraFrame(scan_,(camera_t)i);

    //filterObjects(cameraFrameScan_,i);
    //@TEST
    testProjection(cameraFrameScan_,(camera_t)i);
  }

  //@TEST
  ROS_INFO("successfully transformed to camera frame");
  sensor_msgs::PointCloud2 cameraFrameScanTESTOutput_;
  pcl::toROSMsg(*cameraFrameScan_, cameraFrameScanTESTOutput_);

  publisher_.publish(cameraFrameScanTESTOutput_);
}

void PointFilter::BoundingBoxCallback(
    const darknet_ros_msgs::BoundingBoxesConstPtr& bounding_box_msg) {
      boundingBoxListCam0 = bounding_box_msg;
}

void PointFilter::ObjectCountCallback(const darknet_ros_msgs::ObjectCountConstPtr& msg) {
  objectFoundCountCam0_ = msg->count;
}


/**************************helper functions**********************************/

PointCloud::Ptr PointFilter::transformToCameraFrame(const PointCloud::Ptr input_scan_,camera_t camera) {

  PointCloud::Ptr out_cloud;
  Eigen::Affine3d transform;

  out_cloud = boost::make_shared<PointCloud>();


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

  pcl::transformPointCloud(*input_scan_, *out_cloud, transform); //(cloud_in, cloud_out, transform)

  return out_cloud;

}

//@TODO Update this function for different images, boundingBox lists and object counts
void PointFilter::filterObjects(const PointCloud::Ptr input_scan_, camera_t camera) {

  beam::Vec2 coords;
  beam::Vec3 point;
  uint16_t u, v;
  uint16_t vmax = imageCam0_->rows;
  uint16_t umax = imageCam0_->cols;
  filteredScan_ = boost::make_shared<PointCloud>();

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

    //check if the point is beyond the edges of the image
    if (u < 0 || v < 0 || u > umax || v > vmax)
      continue;

    bool filterPoint = false;

    //check if the point is within a bounding box
    for (uint8_t j = 0; j < objectFoundCountCam0_; j++) {
      if (boundingBoxListCam0 = NULL)
        continue;
      if (u > boundingBoxListCam0->bounding_boxes[j].xmin && u < boundingBoxListCam0->bounding_boxes[j].xmax &&
          v > boundingBoxListCam0->bounding_boxes[j].ymin && v < boundingBoxListCam0->bounding_boxes[j].ymax)
          filterPoint = true;
    }

    //if the point is not in any bounding boxes, push it to filtered point cloud
    if (!filterPoint)
      filteredScan_->push_back(cameraFrameScan_->points[i]);
    }

    //@ note will need to transform camera's filtered scan back into the lidar frame before writing to a common filtered scan in that frame of reference

    pcl::toROSMsg(*filteredScan_, filteredScanOutput_);

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
