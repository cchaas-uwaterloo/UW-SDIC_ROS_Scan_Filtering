# Point_filter Package

## Overview
This package provides a set of tools to filter vehicles and other unwanted objects from 3D LiDAR scans using computer vision. Intended use is for post processing of scans collected during infrastructure inspections. As it is currently configured, this package can provide filtering functionality for robot configurations with one LiDAR scanner and a six camera ladybug camera system. Limited development would be required to reconfigure the package for different sensor configurations.

The package includes two ROS nodes for input buffering and cloud filtering respectively. The darknet ros package interacts with these two nodes and provides object detection functionality within image frames. 

The package is designed for use in a real-time environment, however current detection accuracy and filtering speeds acheivable on the test machine are such that this is not practical at the moment. Further development is required to be able to acheive real-time filtering. 

## Function 
This package includes two nodes that work in tandem to remove unwanted objects from each LiDAR scan collected by a robot as it transits a structure. The point_filter_node (filter node) actually executes the filtering of each point cloud given that cloud and detection data from darknet for a corresponding set of images. The point_filter_buffer (buffer node) receives the LiDAR and camera data and controls its flow to the filter node to maintain synchronization between the input data streams. The ultimate result of the filtering process is a compiled point map containing only structural elements and no vehicles or other distractions. 

Within the package, the point_filter_node filters objects from point clouds by comparing the 2D projection of each point on a camera plane to the image collected by that camera at the same (roughly) instant. To do this, the point cloud collected in the LiDAR reference frame is transformed to the relevant camera frame. Then, each point is projected to the image plane of that camera using the ladybug camera model projection method. Points that project within bounding boxes identified by darknet around objects of interest in that image are removed from the point cloud. This process is repeated for each ladybug camera until all unwanted objects surrounding the robot in each camera image have been filtered from the point cloud. This entire process is further repeated for each point cloud and image set collected by the robot as it transits a structure. 

The point_filter_buffer node supports the point_filter_node by regulating and organizing the flow of image and point cloud messages from the LiDAR and cameras. Ladybug camera images received on separate topics for each camera are serialized for processing by darknet on a single topic. Furthermore, images that do not align temporally with a LiDAR scan are discarded to reduce processing time. Received images and point clouds are stored in order in FIFO queues until they are published to darknet or the filter node respectively. The buffer synchronizes the point cloud and image data provided to the filter node (via darknet in the case of the image data) via a series of handshakes that control when it publishes these messages from its storage queues. This ensures that the filter node is always operating on point clouds and images that were collected at the same time. The synchronization sequence for one set of point cloud and image data (one point cloud and a set of images recorded at the same time) is as follows: 

                        BUFFER                            DARKNET                            FILTER
                        ------                            -------                            ------
                 if prev. cloud and 
                 image processed:                      
                 - pop image from front
                   of storage queue &     ------->    Process image &
                   send to darknet                 Publish detection data       
                 - send camera expected                     .
                   message to filter      ------------------------------------------>   set current camera
                 - set prev image                           .                                indexer
                   processed == false                       .                                
                                                            .
                                                            .          ------->         read detection data into 
                                                                                         current camera index

                 Return to start          <------------------------------------------    send camera info ack. 
           (prev image processed == true)                                                   message to buffer
                ------------------------------------------------------------------------------------------------- ^repeat until 6 images analysed 

                if 6 images and prev. 
                cloud processed
                - pop cloud from front
                  of storage queue & 
                  send to filter node    ------------------------------------------>    load in point cloud and 
                - set prev. cloud                                                       filter on all 6 images
                  processed == false                                                              .
                                                                                                  .
                                                                                                  .     --------------> Publish filtered cloud
                                                                                        send cloud filtered ack.
                Return to start          <------------------------------------------       message to buffer
           (prev cloud processed == true)

                ================================================================================================= ^^repeat for all point clouds 

## Intsructions for use
To run this package the following libraries/packages are required: 
 - libbeam
 - darknet_ros
 - image_transport
 - PCL

The two nodes of the point_filter package and darknet_ros can be started indepently or they may all be started together using he point_filter.launch file. 
If the nodes are started independently, the point_filter_buffer node must be started with the following argument: 

_image_transport:=compressed (if images are being provided via a compressed image topic)

The next section describes the input and output interface topics required to work with this package.

## ROS Topic Interfaces 
The topics described below are default values, they can be modified in the config.yaml file provided with this package

**Subscribed topics:** 

/ladybug/camera0/image_color/compressed [sensor_msgs/CompressedImage]
    ladybug camera 0 image topic

/ladybug/camera1/image_color/compressed [sensor_msgs/CompressedImage]
    ladybug camera 1 image topic

/ladybug/camera2/image_color/compressed [sensor_msgs/CompressedImage]
    ladybug camera 2 image topic

/ladybug/camera3/image_color/compressed [sensor_msgs/CompressedImage]
    ladybug camera 3 image topic

/ladybug/camera4/image_color/compressed [sensor_msgs/CompressedImage]
    ladybug camera 4 image topic

/ladybug/camera5/image_color/compressed [sensor_msgs/CompressedImage]
    ladybug camera 5 image topic

/front/velodyne_points [sensor_msgs/PointCloud2]
    LiDAR raw point cloud topic

**Published topics**

/publish_filtered_topic [sensor_msgs/PointCloud2]
    filtered point cloud topic with unwanted objects removed

/publish_original_topic [sensor_msgs/PointCloud2]
    original unfiltered point cloud topic 

Further topics for communication between the two point_filter nodes and darknet_ros can be viewed and modified in the config.yaml file.

## Outstanding issues

**Issue:** Images processed by darknet at low rate 
**Description:** Running on the NVIDIA GPU, darknet is capable of processing images at around 20 fps. However, in the current configuration
                 images are sent to darknet and processed at around 1 fps. This is likely due to the publishing and acknowledgement protocols
                 running in the point_filter nodes. Optimizing this process to fully exploit the speeds that darknet is capable of would 
                 significantly speed up filtering process. 

**Issue** Darknet unable to identify vehicles in consecutive images
**Description:** Likely due to poor lighting conditions, darknet is sometimes unable to identify a vehicle in many consective images and these 
                 vehicles are not filtered from the corresponding scans. 

**Issue** Pillars in front of vehicles filtered from scans 
**Description:** When darknet identifies a vehicle behind a pillar or other structural element, a bounding bow is provided around the vehicle and 
                 the pillar in the image. This causes the filter node to remove the pillar from that scan. If the pillar is filtered from enough
                 scans, it could be removed from the final map of the structure. 

**Issue** Inaccurate extrinsic calibrations between LiDAR and camera frames
**Description:** From a visual inspection, the calibrations between the LiDAR frame and the cameras seems to be off by several degrees about the z
                 axis (upward from the robot). This results in areas of filtering that are just slightly offset from the object to be filtered despite
                 a good bounding box fit provided by darknet. These extrinsic calibrations likely change with the configuration of the robot and those 
                 used in the point filter should just match the configuration of the robot when the data is collected.




