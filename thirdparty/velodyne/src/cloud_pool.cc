/**************************************************************
 * Copyright (C) 2015-2020, idriverplus(Beijing ZhiXingZhe Inc.)
 * NodeName: velodyne_pointcloud
 * FileName: cloud_pool.cc
 * Description:
 * 1.
 * 2.
 * 3.
 * History:
 * <author>    <time>    <version>    <description>
 * chendong   2018/12/19   1.0.0      bulid this module
 ************************************************************/
#include "cloud_pool.h"

#include <cstring>

namespace driver {
namespace velodyne {

CloudPool::CloudPool() : initialized_(false) {}

CloudPool::CloudPool(const std::string& topic)
    : initialized_(true),
      topic_(topic),
      cloud_ptr_(nullptr),
      index_ptr_(nullptr),
      cloud_index_(-1),
      cloud_shm_(new ShmWrapper(
          topic, sizeof(PointCloudStruct) * kMaxShmBufferSize, false)),
      index_shm_(new ShmWrapper(topic, sizeof(int8_t), true)) {}

CloudPool::CloudPool(CloudPool&& other)
    : initialized_(other.initialized_),
      topic_(std::move(other.topic_)),
      cloud_ptr_(other.cloud_ptr_),
      index_ptr_(other.index_ptr_),
      cloud_index_(other.cloud_index_),
      cloud_shm_(std::move(other.cloud_shm_)),
      index_shm_(std::move(other.index_shm_)) {
  other.cloud_ptr_ = nullptr;
  other.index_ptr_ = nullptr;
  other.cloud_index_ = -1;
}

CloudPool& CloudPool::operator=(CloudPool&& other) {
  if (this != &other) {
    this->initialized_ = other.initialized_;
    this->topic_.swap(other.topic_);
    this->cloud_ptr_ = other.cloud_ptr_;
    other.cloud_ptr_ = nullptr;
    this->index_ptr_ = other.index_ptr_;
    other.index_ptr_ = nullptr;
    this->cloud_index_ = other.cloud_index_;
    other.cloud_index_ = -1;
    this->cloud_shm_ = std::move(other.cloud_shm_);
    this->index_shm_ = std::move(other.index_shm_);
  }

  return *this;
}

CloudPool::~CloudPool() {}

void CloudPool::InitPool() {
  if (cloud_shm_->GetShmID() == -1) {
    cloud_shm_->CreatMemory();
  }
  if (index_shm_->GetShmID() == -1) {
    index_shm_->CreatMemory();
  }
  cloud_shm_->GetMemoryPtr((void**)(&cloud_ptr_));
  index_shm_->GetMemoryPtr((void**)(&index_ptr_));
}

void CloudPool::DeletePool() {
  cloud_shm_->DeleteMemory();
  index_shm_->DeleteMemory();
}

void CloudPool::SetCloud(const pcl::PointCloud<PointCloudShared>& pcl_cloud) {
  UpdateCloudIndex();
  fromPCL2Struct(pcl_cloud, cloud_ptr_ + cloud_index_);
  *index_ptr_ = cloud_index_ + 1;
}

int CloudPool::GetCloud(sensor_msgs::PointCloud2* ros_cloud) {
  pcl::PointCloud<PointCloudShared> pcl_cloud;
  int ret = GetCloud(&pcl_cloud);
  if (ret < 0) {
    return ret;
  }
  pcl::toROSMsg(pcl_cloud, *ros_cloud);

  return 0;
}

int CloudPool::GetCloud(pcl::PointCloud<PointCloudShared>* pcl_cloud) {
  if (cloud_index_ == -1) {
    if (cloud_shm_->GetMemoryPtr((void**)(&cloud_ptr_)) < 0 ||
        index_shm_->GetMemoryPtr((void**)(&index_ptr_)) < 0) {
      return -1;
    } else {
      cloud_index_ = 0;
    }
  }
  if (*index_ptr_ <= 0 || *index_ptr_ > kMaxShmBufferSize ||
      *index_ptr_ == cloud_index_) {
    return -2;
  }
  PointCloudStruct* curr_cloud_ptr = cloud_ptr_ + (*index_ptr_ - 1);
  if (curr_cloud_ptr->width == 0 || curr_cloud_ptr->height == 0) {
    return -3;
  }
  fromStruct2PCL(*curr_cloud_ptr, pcl_cloud);
  cloud_index_ = *index_ptr_;

  return 0;
}

void CloudPool::UpdateCloudIndex() {
  cloud_index_ = (cloud_index_ + 1) % kMaxShmBufferSize;
}

void CloudPool::fromPCL2Struct(
    const pcl::PointCloud<PointCloudShared>& pcl_cloud,
    PointCloudStruct* str_cloud) const {
  fromPCL2Struct(pcl_cloud.header, &(str_cloud->header));
  str_cloud->width = pcl_cloud.width;
  str_cloud->height = pcl_cloud.height;
  str_cloud->is_dense = pcl_cloud.is_dense;
  for (size_t i = 0; i < 4; ++i) {
    str_cloud->sensor_origin[i] = pcl_cloud.sensor_origin_(i);
    // x,y,z,w
    str_cloud->sensor_orientation[i] =
        pcl_cloud.sensor_orientation_.coeffs()(i);
  }
  for (size_t i = 0; i < pcl_cloud.size(); ++i) {
    str_cloud->points[i] = pcl_cloud.points[i];
  }
}
void CloudPool::fromStruct2PCL(
    const PointCloudStruct& str_cloud,
    pcl::PointCloud<PointCloudShared>* pcl_cloud) const {
  fromStruct2PCL(str_cloud.header, &(pcl_cloud->header));
  pcl_cloud->width = str_cloud.width;
  pcl_cloud->height = str_cloud.height;
  pcl_cloud->is_dense = str_cloud.is_dense;
  for (size_t i = 0; i < 4; ++i) {
    pcl_cloud->sensor_origin_(i) = str_cloud.sensor_origin[i];
  }
  // Quaternionf(w,x,y,z)
  pcl_cloud->sensor_orientation_ = Eigen::Quaternionf(
      str_cloud.sensor_orientation[3], str_cloud.sensor_orientation[0],
      str_cloud.sensor_orientation[1], str_cloud.sensor_orientation[2]);
  size_t points_size = str_cloud.width * str_cloud.height;
  pcl_cloud->resize(points_size);
  for (size_t i = 0; i < points_size; ++i) {
    pcl_cloud->points[i] = str_cloud.points[i];
  }
}
void CloudPool::fromPCL2Struct(const pcl::PCLHeader& pcl_header,
                               PointCloudStruct::Header* str_header) const {
  str_header->seq = pcl_header.seq;
  str_header->stamp = pcl_header.stamp;
  memcpy(str_header->frame_id, pcl_header.frame_id.c_str(),
         pcl_header.frame_id.size());
}
void CloudPool::fromStruct2PCL(const PointCloudStruct::Header& str_header,
                               pcl::PCLHeader* pcl_header) const {
  pcl_header->seq = str_header.seq;
  pcl_header->stamp = str_header.stamp;
  pcl_header->frame_id.assign(str_header.frame_id);
}
}  // namespace velodyne
}  // namespace driver
