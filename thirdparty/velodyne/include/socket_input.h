#ifndef DRIVER_VELODYNE_SOCKET_INPUT_H
#define DRIVER_VELODYNE_SOCKET_INPUT_H

// ROS
#include <ros/console.h>
#include <ros/ros.h>

#include <arpa/inet.h>
#include <errno.h>
#include <fcntl.h>
#include <memory.h>
#include <netinet/in.h>
#include <poll.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/file.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <unistd.h>  // close()

#include <iostream>
#include <string>

#include <boost/shared_ptr.hpp>

#include "custom_log.h"
#include "diagnostic_constant.h"
#include "input.h"
#include "packet_data.h"
#include "velodyne_constant.h"

namespace driver {
namespace velodyne {

/** @brief Live Velodyne input from socket. */
class SocketInput : public Input {
 public:
  SocketInput(std::string device_ip_str, uint16_t port);
  virtual ~SocketInput();
  int Init() override;
  int ReceivePacket(void* data, const int length, uint64_t* nsec) override;

 private:
  bool input_available(int timeout);

 private:
  int sockfd_;
  std::string ip_str_;
  uint16_t port_;
  // linux socket api params
  in_addr devip_;
  sockaddr_in my_addr_;  // my address information
};

}  // namespace velodyne
}  // namespace driver

#endif  // DRIVER_VELODYNE_SOCKET_INPUT_H
