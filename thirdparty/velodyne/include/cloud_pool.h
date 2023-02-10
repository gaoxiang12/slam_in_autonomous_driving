/**************************************************************
 * Copyright (C) 2015-2020, idriverplus(Beijing ZhiXingZhe Inc.)
 * NodeName: velodyne_pointcloud
 * FileName: cloud_pool.h
 * Description:
 * 1.
 * 2.
 * 3.
 * History:
 * <author>    <time>    <version>    <description>
 * chendong   2018/12/19   1.0.0      bulid this module
 ************************************************************/
#ifndef PERCEPTION_VELODYNE_POINTCLOUD_CLOUD_POOL_H_
#define PERCEPTION_VELODYNE_POINTCLOUD_CLOUD_POOL_H_

#include <stdint.h>

#include <pcl_ros/point_cloud.h>

#include "point_types.h"
#include "shm_util.h"

namespace driver {
namespace velodyne {

using PointCloudShared = PointXYZRRIAR;

struct PointCloudStruct {
  struct Header {
    uint32_t seq;
    uint64_t stamp;
    char frame_id[256];
  };
  Header header;
  // PointCloudShared points[36000];
  PointCloudShared points[60000];
  uint32_t width;
  uint32_t height;
  bool is_dense;
  float sensor_origin[4];
  float sensor_orientation[4];
};

class CloudPoolInterface;
class CloudWrapper;

class CloudPool {
  friend class CloudPoolInterface;
  friend class CloudWrapper;

 private:
  explicit CloudPool();
  explicit CloudPool(const std::string& topic);
  //  noncopyable
  CloudPool(const CloudPool&) = delete;
  CloudPool& operator=(const CloudPool&) = delete;
  //  movable
  CloudPool(CloudPool&&);
  CloudPool& operator=(CloudPool&&);
  //
  ~CloudPool();

  void SetCloud(const pcl::PointCloud<PointCloudShared>& pcl_cloud);
  void InitPool();
  void DeletePool();

  int GetCloud(sensor_msgs::PointCloud2* ros_cloud);
  int GetCloud(pcl::PointCloud<PointCloudShared>* pcl_cloud);

  void UpdateCloudIndex();

  void fromPCL2Struct(const pcl::PointCloud<PointCloudShared>& pcl_cloud,
                      PointCloudStruct* str_cloud) const;
  void fromStruct2PCL(const PointCloudStruct& str_cloud,
                      pcl::PointCloud<PointCloudShared>* pcl_cloud) const;
  void fromPCL2Struct(const pcl::PCLHeader& pcl_header,
                      PointCloudStruct::Header* str_header) const;
  void fromStruct2PCL(const PointCloudStruct::Header& str_header,
                      pcl::PCLHeader* pcl_header) const;

 private:
  bool initialized_;

  std::string topic_;
  PointCloudStruct* cloud_ptr_;
  int8_t* index_ptr_;
  int8_t cloud_index_;

  std::unique_ptr<ShmWrapper> cloud_shm_;
  std::unique_ptr<ShmWrapper> index_shm_;
};
}  // namespace velodyne
}  // namespace driver

#endif  // PERCEPTION_VELODYNE_POINTCLOUD_CLOUD_POOL_H_
