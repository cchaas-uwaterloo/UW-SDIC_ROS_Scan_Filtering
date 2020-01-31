#include "point_filter/point_filter.h"

namespace point_filter {

PointFilter::PointFilter(ros::NodeHandle& node_handle)
    : nh_(node_handle), nh_priv_("~") {
  if (!ReadParameters()) { ROS_ERROR("Error reading ROS parameters"); }

  publisher_ = nh_priv_.advertise<sensor_msgs::PointCloud2>(publish_topic_, 1);

  publisher_ack_ = nh_priv_.advertise<std_msgs::Int16>(publish_ack_topic_, 1);
  publisher_pclack_ = nh_priv_.advertise<std_msgs::Int16>(publish_pclack_topic_, 1);

  //@TEST
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
  ack_m.data = 1;
  imageSem_ = 0;
  filterIntensity_ = 1.5;
  acknowledgeSent = true;
  firstImages_ = 2;

  bounding_box_subscriber_ = nh_.subscribe(bounding_box_topic_, 1,
                              &PointFilter::BoundingBoxCallback, this);
  object_count_subscriber_ = nh_.subscribe(object_count_topic_, 1,
                              &PointFilter::ObjectCountCallback, this);
  image_subscriber_ = nh_.subscribe(image_topic_, 1,
                              &PointFilter::ImageCallback, this);
  lidar_subscriber_ = nh_.subscribe(lidar_topic_, 1,
                              &PointFilter::LidarCallback, this);

  camera_number_subscriber_ = nh_.subscribe(camera_number_topic_, 1,
                              &PointFilter::CameraNumberCallback, this);

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
                              "/darknet_ros/detection_image");
  nh_priv_.param<std::string>("lidar_topic", lidar_topic_,
                              "/point_cloud");

  nh_priv_.param<std::string>("camera_number_topic", camera_number_topic_,
                              "/camera_number");
  nh_priv_.param<std::string>("publish_ack_topic", publish_ack_topic_,
                              "/camera_acknowledge");
  nh_priv_.param<std::string>("publish_pclack_topic", publish_pclack_topic_,
                              "/pcl_acknowledge");


  nh_priv_.param<std::string>("publish_topic_test", publish_topic_test_,
                              "/test_original_cloud");

  ROS_INFO("bounding_box_topic: %s", bounding_box_topic_.c_str());
  ROS_INFO("publish_topic: %s", publish_topic_.c_str());
  ROS_INFO("publish_ack_topic: %s", publish_ack_topic_.c_str());
  ROS_INFO("image_topic: %s", image_topic_.c_str());
  ROS_INFO("lidar_topic: %s", lidar_topic_.c_str());
  return true;
}

bool PointFilter::LoadIntrinsics() {
  std::string file_location = __FILE__;
  file_location.erase(file_location.end() - 28, file_location.end());
  file_location += "/point_filter/config/ladybug.conf";
  ROS_INFO("Openning Intrinsics: %s", file_location.c_str());

  for (int i = 0; i < 6; i++) {
      intrinsics_[i] = std::make_shared<beam_calibration::LadybugCamera> (i,file_location);
  }

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

  if (!imageIsEqual(cv_bridge::toCvCopy(image_msg, sensor_msgs::image_encodings::BGR8)->image,lastImage_) && firstImages_ == 0 ) {
    ROS_INFO("Loading camera %d", reading_camera_);
    imageCam_[reading_camera_] = cv_bridge::toCvCopy(image_msg, sensor_msgs::image_encodings::BGR8)->image;
    lastImage_ = imageCam_[reading_camera_];
    imageSem_ = 2;
    acknowledgeSent = false;
  }
  else {
    ROS_INFO("discarded image read");
    if (firstImages_ > 0) {
      lastImage_ = cv_bridge::toCvCopy(image_msg, sensor_msgs::image_encodings::BGR8)->image;
      firstImages_ --;
    }
  }

}

void PointFilter::LidarCallback(
    const sensor_msgs::PointCloud2& lidar_msg) {

  //save the header of the input cloud to write back to the output cloud
  std_msgs::Header inputCloudHeader = lidar_msg.header;

  //convert ROS message to point cloud type
  PointCloud2::Ptr  scan_tmp_ (new PointCloud2);
  pcl_conversions::toPCL(lidar_msg, *scan_tmp_);
  scan_ = boost::make_shared<PointCloud>();
  filteredScanLidarInv_ = boost::make_shared<PointCloud>();
  pcl::fromPCLPointCloud2(*scan_tmp_, *scan_);

  for (int i=camera_t::lbcam0; i <= camera_t::lbcam5; i++) {
    ROS_INFO("Filtering on camera %d", i-1);
    scan_ = filterObjects(scan_,(camera_t)i);
  }

  pcl::toROSMsg(*scan_, scanOutput_);

  pcl::toROSMsg(*filteredScanLidarInv_, scanOutputInv_);

  //write the header information from the input cloud to the ouput cloud
  scanOutput_.header = inputCloudHeader;

  publisher_.publish(scanOutput_);
  publisher_test.publish(lidar_msg);

  //acknowledge pcl filtered to point cloud buffer (prompts to send )
  publisher_pclack_.publish(ack_m);
  ROS_INFO("Sent PCL processed acknowledgement");
}

void PointFilter::BoundingBoxCallback(const darknet_ros_msgs::BoundingBoxesConstPtr& bounding_box_msg) {
  if (imageSem_ > 0) {
    if (bounding_box_msg != NULL)
      boundingBoxListCam_[reading_camera_] = *bounding_box_msg;
    ROS_INFO("Loaded bounding boxes from camera %d", reading_camera_);
    imageSem_ --;
  }
  if (imageSem_ == 0 && !acknowledgeSent) {
    publisher_ack_.publish(ack_m);
    ROS_INFO("Acknowledged camera %d image - all data recieved", reading_camera_);
    acknowledgeSent = true;
  }


}

void PointFilter::ObjectCountCallback(const darknet_ros_msgs::ObjectCountConstPtr& msg) {
  if (imageSem_ > 0) {
    objectFoundCountCam_[reading_camera_] = msg->count;
    ROS_INFO("Loaded object found count from camera %d. %d objects found.", reading_camera_, objectFoundCountCam_[reading_camera_]);
    imageSem_ --;

    //if the object found count is zero, darknet does not publish to the bounding box list topic so the Semaphore must be decremented twice here
    if (msg->count == 0)
      imageSem_ --;
  }
  if (imageSem_ == 0 && !acknowledgeSent){
    publisher_ack_.publish(ack_m);
    ROS_INFO("Acknowledged camera %d image - all data recieved", reading_camera_);
    acknowledgeSent = true;
  }
}

void PointFilter::CameraNumberCallback(const std_msgs::Int16 msg) {
  reading_camera_ = msg.data;
  ROS_INFO("Prepared to read camera %d info", reading_camera_);
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

  transform = transform.inverse();

  pcl::transformPointCloud(*input_scan, *out_scan, transform); //(cloud_in, cloud_out, transform)

}

PointCloud::Ptr PointFilter::filterObjects(PointCloud::Ptr input_scan, camera_t camera) {

  //set camera variables based on camera param
  uint16_t vmax = imageCam_[(int)camera - 1].rows;
  uint16_t umax = imageCam_[(int)camera - 1].cols;
  darknet_ros_msgs::BoundingBoxes bbList = boundingBoxListCam_[(int)camera - 1];
  uint8_t oFCount = objectFoundCountCam_[(int)camera - 1];
  std::vector<beam::Vec2> projectedPoints;

  ROS_INFO("Detected %d objects in camera %d image", oFCount, (int)camera-1);

  PointCloud::Ptr cameraFrameScan_ = boost::make_shared<PointCloud>();
  PointCloud::Ptr filteredScanCamera_ = boost::make_shared<PointCloud>();
  PointCloud::Ptr filteredScanLidar_ = boost::make_shared<PointCloud>();

  PointCloud::Ptr filteredScanCameraInv_ = boost::make_shared<PointCloud>();

  //Two dimensional vector of points found in each bounding box
  std::vector<std::vector<pcl::PointXYZ>> bbPoints(oFCount,std::vector<pcl::PointXYZ>(0));

  //make a copy of the input scan in the camera frame
  transformToCameraFrame(input_scan, cameraFrameScan_, camera);

  transformToCameraFrame(filteredScanLidarInv_, filteredScanCameraInv_, camera);

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
    coords = intrinsics_[(int)camera-1]->ProjectPoint(point);

    projectedPoints.push_back(coords);

    u = std::round(coords(0, 0));
    v = std::round(coords(1, 0));

    //check if the point is beyond the edges of the image
    if (u < 0 || v < 0 || u > umax || v > vmax)
      beyond_plane = true;

    bool filterPoint = false;

    //check if the point is within a bounding box
    for (uint8_t j = 0; j < oFCount; j++) {

      int xmin = bbList.bounding_boxes[j].xmin;
      int xmax = bbList.bounding_boxes[j].xmax;
      int ymin = bbList.bounding_boxes[j].ymin;
      int ymax = bbList.bounding_boxes[j].ymax;

      //Only filter if that bounding box describes the bounds of a car or truck
      if (bbList.bounding_boxes[j].Class == "car" || bbList.bounding_boxes[j].Class == "truck") {
        if (u > xmin && u < xmax && v > ymin && v < ymax) {
              filterPoint = true;
              bbPoints[j].push_back(cameraFrameScan_->points[i]);
        }
      }

    }

      //if the point is not in any bounding boxes, push it to filtered point cloud
      if (!filterPoint || behind_plane || beyond_plane)
        filteredScanCamera_->push_back(cameraFrameScan_->points[i]);

      //if the point is in a bounding box, push it to the inverted point cloud
      else if (filterPoint)
        filteredScanCameraInv_->push_back(cameraFrameScan_->points[i]);
  }

  //restore outliers from filtered scan
  restoreOutliers(filteredScanCamera_,oFCount,bbPoints,filterIntensity_);

  //transform the filtered scan in the camera frame back to the lidar frame
  transformToLidarFrame(filteredScanCamera_, filteredScanLidar_, camera);

  transformToLidarFrame(filteredScanCameraInv_, filteredScanLidarInv_, camera);

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
    coords = intrinsics_[(int)camera-1]->ProjectPoint(point);

    u = std::round(coords(0, 0));
    v = std::round(coords(1, 0));

    ROS_INFO ("The projected U coordinate of point %d is %d in camera %d", i, u, camera);
    ROS_INFO ("The projected V coordinate of point %d is %d in camera %d", i, v, camera);


  }

}

bool PointFilter::imageIsEqual(const cv::Mat mat1, const cv::Mat mat2){

  ROS_INFO("comparing images.");

   // treat two empty mat as identical as well
   if (mat1.empty() && mat2.empty()) {
       return true;
   }
   // if dimensionality of two mat is not identical, these two mat are not identical
   if (mat1.cols != mat2.cols || mat1.rows != mat2.rows || mat1.dims != mat2.dims) {
       return false;
   }
   cv::Mat diff, diff_grey;
   cv::compare(mat1, mat2, diff, cv::CMP_NE);
   cv::cvtColor(diff, diff_grey, CV_BGR2GRAY);
   int nz = cv::countNonZero(diff_grey);
   return nz==0;
}

void PointFilter::restoreOutliers(PointCloud::Ptr& cloud, uint8_t objectCount, const std::vector<std::vector<pcl::PointXYZ>>& bbPoints, float factor) {

  //for each bounding box, find the mean and standard deviation of z values of the points therein and push all the outlying points back to the filtered point cloud
  for (uint8_t i = 0; i < objectCount; i++) {

    float meanZ, varianceZ, stdDevZ = 0;

    //determine the mean value of the z values
    for (uint16_t j = 0; j < bbPoints[i].size(); j++) {
      meanZ += bbPoints[i][j].z;
    }
    meanZ /= (bbPoints[i].size() != 0 ? bbPoints[i].size() : 1);

    //determine the variance of the z values
    for (uint16_t k = 0; k < bbPoints[i].size(); k++) {
      varianceZ += pow(bbPoints[i][k].z - meanZ, 2);
    }
    varianceZ /= (bbPoints[i].size() != 0 ? bbPoints[i].size() : 1);
    stdDevZ = sqrt(varianceZ);

    //restore any point more than 1 std dev from the mean in the z axis
    for (uint16_t l = 0; l < bbPoints[i].size(); l++) {
      if (fabs(bbPoints[i][l].z - meanZ) > fabs(stdDevZ*factor - meanZ))
        cloud->push_back(bbPoints[i][l]);
    }

  }

}

//test function to visualize point clouds
void PointFilter::visualizer(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, std::string displayName) {
  pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer (displayName));
  viewer->setBackgroundColor (0, 0, 0);
  viewer->addPointCloud (cloud, "sample cloud");
  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud");
  viewer->addCoordinateSystem (1);
  viewer->spin();
}

void PointFilter::displayProjectionImage(camera_t camera, const cv::Mat image, std::vector<beam::Vec2> projectedPoints) {
  cv::Mat local_image = image;
  cv::Scalar black( 0, 0, 0 );
  cv::Scalar red( 0, 255, 0 );
  int u = 0;
  int v = 0;

  darknet_ros_msgs::BoundingBoxes bbList = boundingBoxListCam_[(int)camera - 1];

  std::string name = "/home/evan/test_images/projections/camera" + std::to_string((int)camera-1) + ".jpeg";

  for (int i = 0; i < projectedPoints.size(); i++) {
    u = std::round(projectedPoints[i](0, 0));
    v = std::round(projectedPoints[i](1, 0));
    cv::circle(local_image, cv::Point(u, v),3,black);
  }

  for(int i = 0; i < objectFoundCountCam_[(int)camera - 1]; i++) {

    int xmin = bbList.bounding_boxes[i].xmin;
    int xmax = bbList.bounding_boxes[i].xmax;
    int ymin = bbList.bounding_boxes[i].ymin;
    int ymax = bbList.bounding_boxes[i].ymax;

    for (int j = 0; j < 10; j++) {
      cv::circle(local_image, cv::Point(std::round(xmin+(xmax-xmin)/10*j),ymin),3,red);
      cv::circle(local_image, cv::Point(std::round(xmin+(xmax-xmin)/10*j),ymax),3,red);
      cv::circle(local_image, cv::Point(xmin,std::round(ymin+(ymax-ymin)/10*j)),3,red);
      cv::circle(local_image, cv::Point(xmax,std::round(ymin+(ymax-ymin)/10*j)),3,red);
    }
  }

  //cv::imshow(std::to_string((int)camera), local_image);
  cv::imwrite(name, local_image);
  cv::waitKey(10);

}

} // namespace point_filter
