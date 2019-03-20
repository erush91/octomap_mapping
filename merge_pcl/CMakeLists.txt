cmake_minimum_required(VERSION 2.8.3)
project(merge_pcl)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

find_package(PCL REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  message_generation
  pcl_ros
  pcl_conversions
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES concat_pcl
#  CATKIN_DEPENDS roscpp rospy std_msgs
#  DEPENDS system_lib
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

add_executable(merge_pcl src/merge_pcl.cpp)
target_link_libraries(merge_pcl ${catkin_LIBRARIES})
