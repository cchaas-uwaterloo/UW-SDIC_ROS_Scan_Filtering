cmake_minimum_required(VERSION 2.8)
project(test_stubs)

find_package(catkin REQUIRED COMPONENTS cv_bridge image_transport message_generation sensor_msgs)

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

include_directories(include ${catkin_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})

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

# add the plugin example
add_library(resized_publisher src/manifest.cpp src/resized_publisher.cpp src/resized_subscriber.cpp)
add_dependencies(resized_publisher ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS})
target_link_libraries(resized_publisher ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})


# Mark executables and/or libraries for installation
install(TARGETS im_publisher im_subscriber resized_publisher bb_subscriber
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(FILES resized_plugins.xml
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
