cmake_minimum_required(VERSION 2.8.3)
project(kinectv2_viewer)

#find_package(Eigen REQUIRED)
#find_package(Boost REQUIRED thread date_time system filesystem program_options python)

# Load catkin and all dependencies required for this package
find_package(catkin REQUIRED
  roscpp
  std_msgs
  pcl_ros
  eigen_conversions 
  tf
  tf_conversions
)

catkin_package(DEPENDS
  roscpp
  std_msgs
  pcl_ros
  eigen_conversions 
  tf
  tf_conversions
#  INCLUDE_DIRS include
#  LIBRARIES ${PROJECT_NAME}
)

# Enable gdb to show line numbers
#SET(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wall -g")
set(CMAKE_BUILD_TYPE Debug)

## Build 
include_directories(
  ${catkin_INCLUDE_DIRS}
)
link_directories(${catkin_LIBRARY_DIRS})
 
add_executable(kinectv2_viewer src/kinectv2_viewer.cpp)
target_link_libraries(kinectv2_viewer ${catkin_LIBRARIES})
