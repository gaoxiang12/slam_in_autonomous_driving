/**************************************************************
 * Copyright (C) 2015-2020, idriverplus(Beijing ZhiXingZhe Inc.)
 * NodeName: velodyne_pointcloud
 * FileName: shm_util.cc
 * Description:
 * 1.
 * 2.
 * 3.
 * History:
 * <author>    <time>    <version>    <description>
 * chendong   2018/12/19   1.0.0      bulid this module
 ************************************************************/
#include "shm_util.h"

#include <iostream>
#include <sstream>

namespace driver {
namespace velodyne {

ShmWrapper::ShmWrapper(const std::string& topic, const size_t shm_size,
                       const bool is_index)
    : shm_size_(shm_size), shmid_(-1) {
  std::stringstream ss(topic.substr(kTopicLetterSize));
  int num;
  ss >> num;
  if (is_index) {
    shm_key_ -= num + 1;
  } else {
    shm_key_ += num + 1;
  }
}

ShmWrapper::ShmWrapper(const key_t shm_key, const size_t shm_size)
    : shm_key_(shm_key), shm_size_(shm_size), shmid_(-1) {}

ShmWrapper::~ShmWrapper() {}

int ShmWrapper::CreatMemory() {
  shmid_ = shmget(shm_key_, shm_size_, IPC_CREAT | 0666);
  if (shmid_ == -1) {
    perror("shmget failed");
    exit(1);
  }
  return shmid_;
}

int ShmWrapper::GetMemoryPtr(void** shared) {
  void* sh = nullptr;
  shmid_ = shmget(shm_key_, 0, 0);
  if (shmid_ == -1) return -1;
  sh = shmat(shmid_, NULL, 0);
  if (sh == (void*)-1) return -2;
  *shared = sh;
  return shmid_;
}

int ShmWrapper::DetachMemoryPtr(void** shared) const {
  if (shmdt(*shared) == -1) return -3;
  return 0;
}
// ipcs -m    ipcrm -m [shmid]
int ShmWrapper::DeleteMemory() const {
  if (shmctl(shmid_, IPC_RMID, 0) == -1) return -4;
  return 0;
}

int ShmWrapper::GetShmID() const { return shmid_; }
}  // namespace velodyne
}  // namespace driver
