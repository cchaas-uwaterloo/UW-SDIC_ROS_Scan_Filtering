cmake_minimum_required(VERSION 2.8)
project(test_stubs)

find_package(beam REQUIRED)
find_package(catkin REQUIRED COMPONENTS cv_bridge image_transport message_generation sensor_msgs)
find_package(darknet_ros_msgs REQUIRED)

# add the resized image message
add_message_files(DIRECTORY msg
   FILES 
   ResizedImage.msg
   BoundingBox.msg 
   BoundingBoxes.msg
)
generate_messages(DEPENDENCIES sensor_msgs)

catkin_package(CATKIN_DEPENDS cv_bridge image_transport message_runtime sensor_msgs)

find_package(OpenCV)

include_directories(include ${catkin_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS} ${darknet_ros_msgs_INCLUDE_DIRS})
message(STATUS ${darknet_ros_msgs_INCLUDE_DIRS})

# add the image publisher
add_executable(im_publisher src/im_publisher.cpp)
add_dependencies(im_publisher ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS})
target_link_libraries(im_publisher ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

# add the image subscriber
add_executable(im_subscriber src/im_subscriber.cpp)
add_dependencies(im_subscriber ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS})
target_link_libraries(im_subscriber ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

# add the bounding box subscriber
add_executable(bb_subscriber src/bb_subscriber.cpp)
add_dependencies(bb_subscriber ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS})
target_link_libraries(bb_subscriber ${catkin_LIBRARIES})

# add the point cloud publisher 
add_executable(pcl_publisher src/pcl_publisher.cpp)
add_dependencies(pcl_publisher ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS})
target_link_libraries(pcl_publisher ${catkin_LIBRARIES} beam::beam)

# add the point cloud subscriber
add_executable(pcl_subscriber src/pcl_subscriber.cpp)
add_dependencies(pcl_subscriber ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS})
target_link_libraries(pcl_subscriber ${catkin_LIBRARIES} beam::beam)

# add the test bag reader
add_executable(test_bag_reader src/test_bag_reader.cpp)
add_dependencies(test_bag_reader ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS})
target_link_libraries(test_bag_reader ${catkin_LIBRARIES} beam::beam)

# Mark executables and/or libraries for installation
install(TARGETS im_publisher im_subscriber bb_subscriber
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(FILES resized_plugins.xml
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
