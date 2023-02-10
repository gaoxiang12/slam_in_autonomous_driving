/**************************************************************
 * Copyright (C) 2015-2020, idriverplus(Beijing ZhiXingZhe Inc.)
 * NodeName: velodyne_pointcloud
 * FileName: shm_util.h
 * Description:
 * 1.
 * 2.
 * 3.
 * History:
 * <author>    <time>    <version>    <description>
 * chendong   2018/12/19   1.0.0      bulid this module
 ************************************************************/
#ifndef PERCEPTION_VELODYNE_POINTCLOUD_SHM_UTIL_H_
#define PERCEPTION_VELODYNE_POINTCLOUD_SHM_UTIL_H_

#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <cstring>
#include <fstream>

#include <sys/ipc.h>
#include <sys/shm.h>
#include <sys/types.h>

namespace driver {
namespace velodyne {

static const int kMaxShmBufferSize = 20;
static const int kTopicLetterSize = 13;

class ShmWrapper {
 public:
  ShmWrapper(const std::string& topic, const size_t shm_size,
             const bool is_index);
  ShmWrapper(const key_t shm_key, const size_t shm_size);
  //  noncopyable
  ShmWrapper(const ShmWrapper&) = delete;
  ShmWrapper& operator=(const ShmWrapper&) = delete;
  ~ShmWrapper();

  int CreatMemory();

  int GetMemoryPtr(void** shared);

  int DetachMemoryPtr(void** shared) const;

  int DeleteMemory() const;

  int GetShmID() const;

 private:
  key_t shm_key_ = 0x600000;
  size_t shm_size_;
  int shmid_;
};
}  // namespace velodyne
}  // namespace driver

#endif  // PERCEPTION_VELODYNE_POINTCLOUD_SHM_UTIL_H_
