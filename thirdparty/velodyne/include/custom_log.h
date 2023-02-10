#ifndef KERNEL_PERCEPTION_DRIVER_VELODYNE_CUSTOM_LOG_H
#define KERNEL_PERCEPTION_DRIVER_VELODYNE_CUSTOM_LOG_H

#include <iostream>

#define DENABLE_PRINT

#ifdef DENABLE_PRINT

#define __STD_IOSTREAM 0
#define __ROS_IOSTREAM 1
#define __GLOG_IOSTREAM 2

#define __LOG_METHOD __ROS_IOSTREAM

#if __LOG_METHOD == __STD_IOSTREAM
#define MY_LOG_INFO(...) std::cout << __VA_ARGS__ << std::endl
#define MY_LOG_INFO_VERBOSE(...) \
  std::cout << "[" << __func__ << "] " << __VA_ARGS__ << std::endl
#define MY_LOG_ERROR(...) \
  std::cout << "\033[31m" << __VA_ARGS__ << "\033[0m" << std::endl;
#define MY_LOG_ERROR_VERBOSE(...)                                  \
  std::cout << "\033[31m"                                          \
            << "[" << __func__ << "] " << __VA_ARGS__ << "\033[0m" \
            << std::endl;
#define MY_LOG_DEBUG(...) \
  std::cout << "\033[33m" << __VA_ARGS__ << "\033[0m" << std::endl;
#define MY_LOG_DEBUG_VERBOSE(...)                                  \
  std::cout << "\033[33m"                                          \
            << "[" << __func__ << "] " << __VA_ARGS__ << "\033[0m" \
            << std::endl;

#elif __LOG_METHOD == __ROS_IOSTREAM
#define MY_LOG_INFO(...) ROS_INFO_STREAM(__VA_ARGS__);
#define MY_LOG_INFO_VERBOSE(...) \
  ROS_INFO_STREAM("[" << __func__ << "] " << __VA_ARGS__)
#define MY_LOG_ERROR(...) ROS_ERROR_STREAM(__VA_ARGS__)
#define MY_LOG_ERROR_VERBOSE(...) \
  ROS_ERROR_STREAM("[" << __func__ << "] " << __VA_ARGS__)
#define MY_LOG_DEBUG(...) ROS_DEBUG_STREAM(__VA_ARGS__)
#define MY_LOG_DEBUG_VERBOSE(...) \
  ROS_DEBUG_STREAM("[" << __func__ << "] " << __VA_ARGS__)

#elif __LOG_METHOD == __GLOG_IOSTREAM

#endif  // __LOG_METHOD

#else
#define MY_LOG_INFO(...)
#define MY_LOG_INFO_VERBOSE(...)
#define MY_LOG_ERROR(...)
#define MY_LOG_ERROR_VERBOSE(...)
#define MY_LOG_DEBUG(...)
#define MY_LOG_DEBUG_VERBOSE(...)

#endif  // DENABLE_PRINT

#endif  // KERNEL_PERCEPTION_DRIVER_VELODYNE_CUSTOM_LOG_H