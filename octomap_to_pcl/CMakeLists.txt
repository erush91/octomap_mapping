cmake_minimum_required(VERSION 2.8.3)
project(octomap_to_pcl)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

find_package(PCL REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  sensor_msgs
  message_generation
  octomap_ros
  octomap_msgs
  pcl_ros
  pcl_conversions
)

find_package(catkin REQUIRED COMPONENTS ${PACKAGE_DEPENDENCIES})

find_package(octomap REQUIRED)

generate_messages(
  DEPENDENCIES
  std_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${OCTOMAP_INCLUDE_DIRS}
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS ${PACKAGE_DEPENDENCIES}
  DEPENDS octomap PCL
)

set(LINK_LIBS
  ${OCTOMAP_LIBRARIES}
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)

add_library(${PROJECT_NAME} src/octomap_to_pcl.cpp)
target_link_libraries(${PROJECT_NAME} ${LINK_LIBS})
add_dependencies(${PROJECT_NAME} ${PROJECT_NAME}_gencfg)

add_executable(convert_oct_pcl_node src/convert_oct_pcl_node.cpp)
target_link_libraries(convert_oct_pcl_node ${PROJECT_NAME} ${LINK_LIBS})
