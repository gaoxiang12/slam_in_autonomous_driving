#ifndef DRIVER_VELODYNE_BAG_INPUT_H
#define DRIVER_VELODYNE_BAG_INPUT_H

// ROS
#include <ros/console.h>
#include <ros/ros.h>

#include <iostream>
#include <string>

#include <boost/shared_ptr.hpp>

#include "velodyne_msgs/VelodyneScan.h"

#include "custom_log.h"
#include "diagnostic_constant.h"
#include "input.h"
#include "packet_data.h"
#include "synchronized_queue.h"
#include "velodyne_constant.h"

namespace driver {
namespace velodyne {

/** @brief Live Velodyne input from PCAP file. */
class BagInput : public Input {
 public:
  BagInput(const std::string packet_topic, bool use_simulation_time);
  virtual ~BagInput();
  int Init() override;
  int ReceivePacket(void* data, const int length, uint64_t* nsec) override;

 private:
  void PacketCallback(const velodyne_msgs::VelodyneScan::ConstPtr& msg);

 private:
  std::string topic_name_;
  bool use_bag_time_;
  ros::Subscriber sub_packets_;
  boost::shared_ptr<SynchronizedQueue<TimeHDLDataPacket>> bag_packets_;
};

}  // namespace velodyne
}  // namespace driver

#endif  // DRIVER_VELODYNE_BAG_INPUT_H
