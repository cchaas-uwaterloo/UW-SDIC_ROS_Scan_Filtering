cmake_minimum_required(VERSION 3.13)
project(point_filter)

set(CMAKE_CXX_STANDARD 17)

set(build_depends
  roscpp
  sensor_msgs
  cv_bridge
  pcl_ros
  darknet_ros_msgs
  geometry_msgs
)

find_package(beam REQUIRED)

find_package(catkin REQUIRED
  COMPONENTS
    ${build_depends}
)

find_package(OpenCV REQUIRED)
find_package(PCL REQUIRED)


catkin_package(
  INCLUDE_DIRS
    include
  LIBRARIES
    ${PROJECT_NAME}
  CATKIN_DEPENDS
    ${build_depends}
)

# Core library
add_library(${PROJECT_NAME}
  src/filter.cpp
)
target_include_directories(${PROJECT_NAME}
  PUBLIC
    include
    ${catkin_INCLUDE_DIRS}
    
)
target_link_libraries(${PROJECT_NAME}
  beam::beam
  ${catkin_LIBRARIES}
)

##Alternative node addition##
# Node - point cloud filter
#add_executable(${PROJECT_NAME}_node
#  src/point_filter_node.cpp
#)

#set_target_properties(${PROJECT_NAME}_node
#  PROPERTIES OUTPUT_NAME node PREFIX ""
#)

#target_link_libraries(${PROJECT_NAME}_node
#  ${PROJECT_NAME}
#)
##################################


# add the point_filter_noe
add_executable(point_filter_node src/point_filter_node.cpp)
add_dependencies(point_filter_node ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS})
target_link_libraries(point_filter_node ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} ${PROJECT_NAME})


# Mark executables and/or libraries for installation
install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.hpp"
)

# Mark other files for installation
install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
