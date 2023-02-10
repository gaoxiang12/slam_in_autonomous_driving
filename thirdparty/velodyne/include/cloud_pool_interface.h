/**************************************************************
 * Copyright (C) 2015-2020, idriverplus(Beijing ZhiXingZhe Inc.)
 * NodeName: velodyne_pointcloud
 * FileName: cloud_pool_interface.h
 * Description:
 * 1.
 * 2.
 * 3.
 * History:
 * <author>    <time>    <version>    <description>
 * chendong   2018/12/19   1.0.0      bulid this module
 ************************************************************/
#ifndef PERCEPTION_VELODYNE_POINTCLOUD_CLOUD_POOL_INTERFACE_H_
#define PERCEPTION_VELODYNE_POINTCLOUD_CLOUD_POOL_INTERFACE_H_

#include "cloud_pool.h"

namespace driver {
namespace velodyne {

class CloudPoolInterface {
 public:
  template <typename CloudType>
  static int GetCloud(CloudType* cloud, const std::string& topic);
};

extern template int
CloudPoolInterface::GetCloud<pcl::PointCloud<PointCloudShared>>(
    pcl::PointCloud<PointCloudShared>* cloud, const std::string& topic);

extern template int CloudPoolInterface::GetCloud<sensor_msgs::PointCloud2>(
    sensor_msgs::PointCloud2* cloud, const std::string& topic);
}  // namespace velodyne

}  // namespace driver

#endif  // PERCEPTION_VELODYNE_POINTCLOUD_CLOUD_POOL_INTERFACE_H_
