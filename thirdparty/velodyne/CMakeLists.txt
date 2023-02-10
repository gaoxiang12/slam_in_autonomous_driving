cmake_minimum_required(VERSION 2.8.3)
project(velodyne)

#set(BOOST_LIBRARYDIR /usr/lib/aarch64-linux-gnu)
set(PCL_DIR /usr/lib/aarch64-linux-gnu/cmake/pcl)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -O0 -g -fopenmp")
find_package(catkin REQUIRED
   COMPONENTS
   roscpp
   pcl_ros
   pcl_conversions
   sensor_msgs
   velodyne_msgs
   common_msgs
)

# boost
find_package(Boost REQUIRED
  COMPONENTS
    thread
    system
)
# PCL
find_package(PCL 1.7 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

# PCAP
find_library(PCAP_LIBRARY pcap DOC "pcap library")
find_path(PCAP_INCLUDE_DIR pcap.h DOC "pcap include directory")
mark_as_advanced(PCAP_LIBRARY PCAP_INCLUDE_DIR)
include_directories(${PCAP_INCLUDE_DIR})

# objects needed by other ROS packages that depend on this one
catkin_package(
   INCLUDE_DIRS include
   LIBRARIES cloud_pool
   CATKIN_DEPENDS roscpp pcl_ros pcl_conversions sensor_msgs
   DEPENDS velodyne_msgs common_msgs
)


include_directories(
  include 
#  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${CMAKE_SOURCE_DIR}/common
  ${CMAKE_SOURCE_DIR}/
)

add_library(cloud_pool 
  src/cloud_pool.cc 
  src/shm_util.cc 
  src/cloud_pool_interface.cc
  src/velodyne_config.cc
)
target_link_libraries(cloud_pool
  ${catkin_LIBRARIES}
)

# boost thread test code




add_executable(cloud_pool_vlp16 
  test/cloud_pool_vlp16.cc
  src/cloud_pool.cc 
  src/shm_util.cc 
  src/cloud_pool_interface.cc
  src/velodyne_config.cc
)
target_link_libraries(cloud_pool_vlp16
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)
add_executable(cloud_pool_vlp32 
  test/cloud_pool_vlp32.cc
  src/cloud_pool.cc 
  src/shm_util.cc 
  src/cloud_pool_interface.cc
  src/velodyne_config.cc
)
target_link_libraries(cloud_pool_vlp32
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)
