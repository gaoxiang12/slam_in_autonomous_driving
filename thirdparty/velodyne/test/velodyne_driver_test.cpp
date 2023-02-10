

#include <iostream>
#include <string>
#include <vector>

#include <pcl_ros/point_cloud.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

#include <boost/shared_ptr.hpp>

void callback(const sensor_msgs::PointCloud2::ConstPtr& msg) {
  ROS_INFO_STREAM("cloud callback");
  ROS_INFO_STREAM(msg->width << " " << msg->height << " " << msg->data.size());
}
int main(int argc, char** argv) {
  ros::init(argc, argv, "velodyne_driver_test");

  ros::NodeHandle node;
  ros::Subscriber sub_cloud = node.subscribe<sensor_msgs::PointCloud2>(
      "/velodyne_pointcloud", 1, callback);

  ros::spin();
}