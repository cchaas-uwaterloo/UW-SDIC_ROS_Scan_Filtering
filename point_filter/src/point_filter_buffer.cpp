#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <std_msgs/Int16.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include <queue>
#include <deque>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/common/common_headers.h>
#include <pcl/features/normal_3d.h>
#include <pcl/io/pcd_io.h>
#include <pcl/console/parse.h>


constexpr float CAMERA_PERIOD = 0.33;
bool cam0Loaded, cam1Loaded, cam2Loaded, cam3Loaded, cam4Loaded, cam5Loaded = false;


struct cameraImage{
  sensor_msgs::ImageConstPtr imagePointer = boost::make_shared<sensor_msgs::Image>();
  std_msgs::Int16 camera;
};

cameraImage operatingImage;

float getTime (const std_msgs::Header& header) {
  return header.stamp.sec + header.stamp.nsec*pow(10,-9);
}

uint16_t imageAcknowledged = 1, pclAcknowledged = 1;
uint16_t imagesProcessedCount = 0;

float lastPCLTime = -1;
sensor_msgs::PointCloud2 holder;

std::deque<sensor_msgs::PointCloud2> pclQueue;
std::queue<cameraImage> imageQueue;

boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> basic_cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>);

std::string lidar_topic, camera_number_topic, acknowledge_topic, pcl_acknowledge_topic,
            camera_reading_topic, subscribers_camera0_topic, subscribers_camera1_topic,
            subscribers_camera2_topic, subscribers_camera3_topic,
            subscribers_camera4_topic, subscribers_camera5_topic,
            subscribers_lidar_raw_topic;

/*
 * @brief function to read in topic names from parameter server
 * @descr tries to read each topic name in from the node parameter server
 *        if it fails to read name from server, sets topic names to default
 *        values
 * @params: None
 * @return: void
 */
bool readParameters();

/*
 * @brief callback function for image acknowledged message
 * @descr sets the local image acknowledged flag when message recieved from
 *        point filter node
 * @params:
 *  msg: acknowledge message from point filter node, sent when that node has
 *       finished processing the previous image provided
 * @return: void
 */
void acknowledgeImageCallback(const std_msgs::Int16 msg);

/*
 * @brief callback function for scan acknowledged message
 * @descr sets the local scan acknowledged flag when message recieved from
 *        point filter node
 * @params:
 *  msg: acknowledge message from point filter node, sent when that node has
 *       finished processing the scan provided
 * @return: void
 */
void acknowledgeScanCallback(const std_msgs::Int16 msg);

/*
 * @brief callback functions for camera n image messages
 * @descr adds received image to buffer queue along with camera n identifier
 *        if the image is suffiently close to the last pcl loaded and the
 *        image for camera n has not already been loaded for that pcl
 * @params:
 *  camera_msg: compressed color image from ladybug camera n
 * @return: void
 */
void camera0Callback (const sensor_msgs::ImageConstPtr& camera_msg);
void camera1Callback (const sensor_msgs::ImageConstPtr& camera_msg);
void camera2Callback (const sensor_msgs::ImageConstPtr& camera_msg);
void camera3Callback (const sensor_msgs::ImageConstPtr& camera_msg);
void camera4Callback (const sensor_msgs::ImageConstPtr& camera_msg);
void camera5Callback (const sensor_msgs::ImageConstPtr& camera_msg);

/*
 * @brief callback function for LiDAR scan messages
 * @descr adds the received pcl to the back of the pcl buffer queue and
 *        resets the image loaded flags. If the previous pcl did not have
 *        all 6 images loaded successfully, discards it from the buffer
 * @params:
 *  pcl_msg: point cloud from lidar scanner
 * @return: void
 */
void cloudCallback (const sensor_msgs::PointCloud2& pcl_msg);



int main(int argc, char** argv) {

  ROS_INFO("Started Node");

  // Initialize current operating image camera data to 0

  operatingImage.camera.data = 0;

  // Read in the topic names from the parameter server

  readParameters();

  // Initalize publishers and subscribers

  ros::init(argc, argv, "point_fiter_buffer");
  ros::NodeHandle nh;

  ros::Publisher pub = nh.advertise<sensor_msgs::Image>(camera_reading_topic, 1);
  ros::Publisher pub2 = nh.advertise<std_msgs::Int16>(camera_number_topic, 1);
  ros::Publisher pub3 = nh.advertise<sensor_msgs::PointCloud2>(lidar_topic, 1);

  image_transport::ImageTransport it(nh);

  image_transport::Subscriber subCam0 = it.subscribe(subscribers_camera0_topic,1, camera0Callback);
  image_transport::Subscriber subCam1 = it.subscribe(subscribers_camera1_topic,1, camera1Callback);
  image_transport::Subscriber subCam2 = it.subscribe(subscribers_camera2_topic,1, camera2Callback);
  image_transport::Subscriber subCam3 = it.subscribe(subscribers_camera3_topic,1, camera3Callback);
  image_transport::Subscriber subCam4 = it.subscribe(subscribers_camera4_topic,1, camera4Callback);
  image_transport::Subscriber subCam5 = it.subscribe(subscribers_camera5_topic,1, camera5Callback);

  ros::Subscriber subPCL = nh.subscribe(subscribers_lidar_raw_topic,1, cloudCallback);

  ros::Subscriber subAckIm = nh.subscribe(acknowledge_topic,1, acknowledgeImageCallback);
  ros::Subscriber subAckPcl = nh.subscribe(pcl_acknowledge_topic,1, acknowledgeScanCallback);

  ROS_INFO("Finished initialization");

  ros::Rate loop_rate(10);
  while (nh.ok()) {

    // Publish the point cloud corresponding to each image set once all 6 images have been processed and acknowledged by the point_filter_node
    // Do not publish the next point cloud until the current one is done processing and the next batch of images is being processed

    if (imagesProcessedCount == 6 && pclAcknowledged == 1) {
      if (!pclQueue.empty()) {
        pub3.publish(pclQueue.front());
        pclQueue.pop_front();
        imagesProcessedCount = 0;
        pclAcknowledged = 0;
        ROS_INFO("Published point cloud for processing");
      }
    }

    // Publish each image with its respective camera designation once the previous image has been acknowledged
    // Do not publish images if the point cloud is being processed and has not been acknowledged

    if (imageAcknowledged == 1 && pclAcknowledged == 1) {
      cameraImage publishingImage;
      if (!imageQueue.empty()) {
        imageAcknowledged = 0;
        publishingImage = imageQueue.front();
        imageQueue.pop();
        pub.publish(publishingImage.imagePointer);
        pub2.publish(publishingImage.camera);
        ROS_INFO("Published %d image to camera/rgb/image_raw topic with Timestamp %f", publishingImage.camera.data, getTime(publishingImage.imagePointer->header));
      }
    }

    ros::spinOnce();
    loop_rate.sleep();
  }
}


bool readParameters() {

  // Set topic names based on config variables

  lidar_topic = ros::param::get("lidar_topic", lidar_topic) ? lidar_topic : "/point_cloud"; // set a default topic name if the param is not retrieved successfully
  camera_number_topic = ros::param::get("camera_number_topic", camera_number_topic) ? camera_number_topic : "/camera_number"; // set a default topic name if the param is not retrieved successfully
  acknowledge_topic = ros::param::get("acknowledge_topic", acknowledge_topic) ? acknowledge_topic : "/camera_acknowledge"; // set a default topic name if the param is not retrieved successfully
  pcl_acknowledge_topic = ros::param::get("pcl_acknowledge_topic", pcl_acknowledge_topic) ? pcl_acknowledge_topic : "/pcl_acknowledge"; // set a default topic name if the param is not retrieved successfully

  camera_reading_topic = ros::param::get("camera_reading_topic", camera_reading_topic) ? camera_reading_topic : "/camera/rgb/image_raw"; // set a default topic name if the param is not retrieved successfully

  subscribers_camera0_topic = ros::param::get("subscribers_camera0_topic", subscribers_camera0_topic) ?
                                subscribers_camera0_topic : "/ladybug/camera0/image_color"; // set a default topic name if the param is not retrieved successfully
  subscribers_camera1_topic = ros::param::get("subscribers_camera1_topic", subscribers_camera1_topic) ?
                                subscribers_camera1_topic : "/ladybug/camera1/image_color"; // set a default topic name if the param is not retrieved successfully
  subscribers_camera2_topic = ros::param::get("subscribers_camera2_topic", subscribers_camera2_topic) ?
                                subscribers_camera2_topic : "/ladybug/camera2/image_color"; // set a default topic name if the param is not retrieved successfully
  subscribers_camera3_topic = ros::param::get("subscribers_camera3_topic", subscribers_camera3_topic) ?
                                subscribers_camera3_topic : "/ladybug/camera3/image_color"; // set a default topic name if the param is not retrieved successfully
  subscribers_camera4_topic = ros::param::get("subscribers_camera4_topic", subscribers_camera4_topic) ?
                                subscribers_camera4_topic : "/ladybug/camera4/image_color"; // set a default topic name if the param is not retrieved successfully
  subscribers_camera5_topic = ros::param::get("subscribers_camera5_topic", subscribers_camera5_topic) ?
                                subscribers_camera5_topic : "/ladybug/camera5/image_color"; // set a default topic name if the param is not retrieved successfully
  subscribers_lidar_raw_topic = ros::param::get("subscribers_lidar_raw_topic", subscribers_lidar_raw_topic) ?
                                  subscribers_lidar_raw_topic : "/front/velodyne_points"; // set a default topic name if the param is not retrieved successfully

  // Write topic names to console

  ROS_INFO("lidar_topic: %s", lidar_topic.c_str());
  ROS_INFO("camera_number_topic: %s", camera_number_topic.c_str());
  ROS_INFO("acknowledge_topic: %s", acknowledge_topic.c_str());
  ROS_INFO("pcl_acknowledge_topic: %s", pcl_acknowledge_topic.c_str());
  ROS_INFO("camera_reading_topic: %s", camera_reading_topic.c_str());
  ROS_INFO("subscribers_camera0_topic: %s", subscribers_camera0_topic.c_str());
  ROS_INFO("subscribers_camera1_topic: %s", subscribers_camera1_topic.c_str());
  ROS_INFO("subscribers_camera2_topic: %s", subscribers_camera2_topic.c_str());
  ROS_INFO("subscribers_camera3_topic: %s", subscribers_camera3_topic.c_str());
  ROS_INFO("subscribers_camera4_topic: %s", subscribers_camera4_topic.c_str());
  ROS_INFO("subscribers_camera5_topic: %s", subscribers_camera5_topic.c_str());
  return true;
}

void acknowledgeImageCallback(const std_msgs::Int16 msg) {
  imageAcknowledged = 1;
  imagesProcessedCount++;
  ROS_INFO("Image acknowledged, imageProcessedCount := %d", imagesProcessedCount);
}

void acknowledgeScanCallback(const std_msgs::Int16 msg) {
  pclAcknowledged = 1;
  ROS_INFO("Received PCL processed acknowledgement");
}


void camera0Callback (const sensor_msgs::ImageConstPtr& camera_msg) {
  float image_time = getTime(camera_msg->header);

  // if the image read in is sufficiently close to the last point cloud read in, enqueue it
  if (image_time-lastPCLTime < CAMERA_PERIOD && image_time >= lastPCLTime &&cam0Loaded == false) {
    operatingImage.imagePointer = camera_msg;
    operatingImage.camera.data = 0;
    imageQueue.push(operatingImage);
    cam0Loaded = true;
    ROS_INFO("Loaded Camera 0 image with timestamp %f image queue at size: %d", image_time, imageQueue.size());
  }
  else
    ROS_INFO("Discared Camera 0 image with timestamp %f", image_time);
}

void camera1Callback (const sensor_msgs::ImageConstPtr& camera_msg) {
  float image_time = getTime(camera_msg->header);

  // if the image read in is sufficiently close to the last point cloud read in, enqueue it
  if (image_time-lastPCLTime < CAMERA_PERIOD && image_time >= lastPCLTime &&cam1Loaded == false) {
    operatingImage.imagePointer = camera_msg;
    operatingImage.camera.data = 1;
    imageQueue.push(operatingImage);
    cam1Loaded = true;
    ROS_INFO("Loaded Camera 1 image with timestamp %f image queue at size: %d", image_time, imageQueue.size());
  }
  else
    ROS_INFO("Discared Camera 1 image with timestamp %f", image_time);
}

void camera2Callback (const sensor_msgs::ImageConstPtr& camera_msg) {
  float image_time = getTime(camera_msg->header);

  // if the image read in is sufficiently close to the last point cloud read in, enqueue it
  if (image_time-lastPCLTime < CAMERA_PERIOD && image_time >= lastPCLTime &&cam2Loaded == false) {
    operatingImage.imagePointer = camera_msg;
    operatingImage.camera.data = 2;
    imageQueue.push(operatingImage);
    cam2Loaded = true;
    ROS_INFO("Loaded Camera 2 image with timestamp %f image queue at size: %d", image_time, imageQueue.size());
  }
  else
    ROS_INFO("Discared Camera 2 image with timestamp %f", image_time);
}

void camera3Callback (const sensor_msgs::ImageConstPtr& camera_msg) {
  float image_time = getTime(camera_msg->header);

  // if the image read in is sufficiently close to the last point cloud read in, enqueue it
  if (image_time-lastPCLTime < CAMERA_PERIOD && image_time >= lastPCLTime &&cam3Loaded == false) {
    operatingImage.imagePointer = camera_msg;
    operatingImage.camera.data = 3;
    imageQueue.push(operatingImage);
    cam3Loaded = true;
    ROS_INFO("Loaded Camera 3 image with timestamp %f image queue at size: %d", image_time, imageQueue.size());
  }
  else
    ROS_INFO("Discared Camera 3 image with timestamp %f", image_time);
}

void camera4Callback (const sensor_msgs::ImageConstPtr& camera_msg) {
  float image_time = getTime(camera_msg->header);

  // if the image read in is sufficiently close to the last point cloud read in, enqueue it
  if (image_time-lastPCLTime < CAMERA_PERIOD && image_time >= lastPCLTime &&cam4Loaded == false) {
    operatingImage.imagePointer = camera_msg;
    operatingImage.camera.data = 4;
    imageQueue.push(operatingImage);
    cam4Loaded = true;
    ROS_INFO("Loaded Camera 4 image with timestamp %f image queue at size: %d", image_time, imageQueue.size());
  }
  else
    ROS_INFO("Discared Camera 4 image with timestamp %f", image_time);
}

void camera5Callback (const sensor_msgs::ImageConstPtr& camera_msg) {
  float image_time = getTime(camera_msg->header);

  // if the image read in is sufficiently close to the last point cloud read in, enqueue it
  if (image_time-lastPCLTime < CAMERA_PERIOD && image_time >= lastPCLTime &&cam5Loaded == false) {
    operatingImage.imagePointer = camera_msg;
    operatingImage.camera.data = 5;
    imageQueue.push(operatingImage);
    cam5Loaded = true;
    ROS_INFO("Loaded Camera 5 image with timestamp %f image queue at size: %d", image_time, imageQueue.size());
  }
  else
    ROS_INFO("Discared Camera 5 image with timestamp %f", image_time);
}

void cloudCallback (const sensor_msgs::PointCloud2& pcl_msg) {

  //if the previous point cloud did not have images loaded for it, erase it from the pclQueue
  if (cam0Loaded == false && cam1Loaded == false && cam2Loaded == false && cam3Loaded == false &&
      cam4Loaded == false && cam5Loaded == false && !pclQueue.empty()) {
    pclQueue.pop_back();
    ROS_INFO("Discarded PCl");
  }

  pclQueue.push_back(pcl_msg);
  lastPCLTime = getTime(pcl_msg.header);
  cam0Loaded = false;
  cam1Loaded = false;
  cam2Loaded = false;
  cam3Loaded = false;
  cam4Loaded = false;
  cam5Loaded = false;
  ROS_INFO("Loaded pcl with timestamp %f", getTime(pcl_msg.header));
}
