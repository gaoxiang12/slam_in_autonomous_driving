/**************************************************************
 * Copyright (C) 2015-2020, idriverplus(Beijing ZhiXingZhe Inc.)
 * NodeName: velodyne_pointcloud
 * FileName: cloud_pool_interface.cc
 * Description:
 * 1.
 * 2.
 * 3.
 * History:
 * <author>    <time>    <version>    <description>
 * chendong   2018/12/20   1.0.0      bulid this module
 ************************************************************/
#include "cloud_pool_interface.h"
#include "custom_log.h"

namespace driver {
namespace velodyne {

const int kMaxCloudTopicNum = 5;

template <typename CloudType>
int CloudPoolInterface::GetCloud(CloudType* cloud, const std::string& topic) {
  int cloud_topic_num;
  if (topic.substr(0, kTopicLetterSize) != "lidar_points_" ||
      (cloud_topic_num = std::stoi(topic.substr(kTopicLetterSize))) >=
          kMaxCloudTopicNum) {
    MY_LOG_ERROR("CloudPoolInterface got a wrong lidar model.");
    return -1;
  }
  static CloudPool cloud_topics[kMaxCloudTopicNum];
  if (!cloud_topics[cloud_topic_num].initialized_) {
    cloud_topics[cloud_topic_num] = std::move(CloudPool(topic));
  }

  return (cloud_topics[cloud_topic_num].GetCloud(cloud));
}

template int CloudPoolInterface::GetCloud<pcl::PointCloud<PointCloudShared>>(
    pcl::PointCloud<PointCloudShared>* cloud, const std::string& topic);

template int CloudPoolInterface::GetCloud<sensor_msgs::PointCloud2>(
    sensor_msgs::PointCloud2* cloud, const std::string& topic);
}  // namespace velodyne
}  // namespace driver
