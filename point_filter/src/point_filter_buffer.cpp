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


/*
* @descr Node that subscribes to the image and pcl topics from the bag file and
         serializes the images for delivery to the point_filter_node
*/

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

int main(int argc, char** argv) {

  ROS_INFO("Started Node");

  //operatingImage.imagePointer = boost::make_shared<sensor_msgs::ImagePtr>();
  operatingImage.camera.data = 0;

  ros::init(argc, argv, "point_fiter_buffer");
  ros::NodeHandle nh;
  ros::Publisher pub = nh.advertise<sensor_msgs::Image>("camera/rgb/image_raw", 1);
  ros::Publisher pub2 = nh.advertise<std_msgs::Int16>("camera_number", 1);
  ros::Publisher pub3 = nh.advertise<sensor_msgs::PointCloud2>("/point_cloud", 1);

  image_transport::ImageTransport it(nh);

  image_transport::Subscriber subCam0 = it.subscribe("/ladybug/camera0/image_color",1, camera0Callback);
  image_transport::Subscriber subCam1 = it.subscribe("/ladybug/camera1/image_color",1, camera1Callback);
  image_transport::Subscriber subCam2 = it.subscribe("/ladybug/camera2/image_color",1, camera2Callback);
  image_transport::Subscriber subCam3 = it.subscribe("/ladybug/camera3/image_color",1, camera3Callback);
  image_transport::Subscriber subCam4 = it.subscribe("/ladybug/camera4/image_color",1, camera4Callback);
  image_transport::Subscriber subCam5 = it.subscribe("/ladybug/camera5/image_color",1, camera5Callback);

  ros::Subscriber subPCL = nh.subscribe("/front/velodyne_points",1, cloudCallback);

  ros::Subscriber subAckIm = nh.subscribe("/camera_acknowledge",1, acknowledgeImageCallback);
  ros::Subscriber subAckPcl = nh.subscribe("/pcl_acknowledge",1, acknowledgeScanCallback);

  ROS_INFO("Finished initialization");

  ros::Rate loop_rate(10);
  while (nh.ok()) {

    //publish the point cloud corresponding to each image set once all 6 images have been processed and acknowledged by the point_filter_node
    //do not publish the next point cloud until the current one is done processing and the next batch of images is being processed
    if (imagesProcessedCount == 6 && pclAcknowledged == 1) {
      if (!pclQueue.empty()) {
        pub3.publish(pclQueue.front());

        //@Test save the raw clouds for later comparison
        pcl::fromROSMsg(pclQueue.front(), *basic_cloud_ptr);
        std_msgs::Header h = pclQueue.front().header;
        std::string timestamp = std::to_string(h.stamp.sec)+ "_" + std::to_string(h.stamp.nsec);
        std::string path = "/home/evan/raw_scans/s_" + timestamp + ".pcd";
        pcl::io::savePCDFileASCII (path, *basic_cloud_ptr);

        pclQueue.pop_front();
        imagesProcessedCount = 0;
        pclAcknowledged = 0;
        ROS_INFO("Published point cloud for processing");
      }
    }

    //publish each image with its respective camera designation once the previous image has been acknowledged
    //do not publish images if the point cloud is being processed and has not been acknowledged
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
