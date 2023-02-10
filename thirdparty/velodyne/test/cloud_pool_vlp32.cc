/**************************************************************
 * Copyright (C) 2015-2020, idriverplus(Beijing ZhiXingZhe Inc.)
 * NodeName: cloud_pool_test
 * FileName: cloud_pool_test.cc
 * Description:
 * 1.
 * 2.
 * 3.
 * History:
 * <author>    <time>    <version>    <description>
 * chendong   2018/12/19   1.0.0      bulid this module
 ************************************************************/
#include <chrono>

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

#include "cloud_pool_interface.h"

using std::chrono::duration;
using std::chrono::duration_cast;
using std::chrono::steady_clock;

using namespace driver::velodyne;

int main(int argc, char** argv) {
  ros::init(argc, argv, "cloud_pool_vlp32");
  ros::NodeHandle node;
  ros::NodeHandle private_nh("~");

  ros::Publisher pub =
      node.advertise<sensor_msgs::PointCloud2>("lidar_points_1", 10);

  sensor_msgs::PointCloud2 ros_cloud;
  ros::Rate rate(100);
  while (ros::ok()) {
    int ret = CloudPoolInterface::GetCloud(&ros_cloud, "lidar_points_1");
    if (ret == 0) {
      pub.publish(ros_cloud);
    }
    rate.sleep();
  }

  return 0;
}