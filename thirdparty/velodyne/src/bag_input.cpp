#include "bag_input.h"

#include <boost/chrono.hpp>
#include <boost/thread/thread.hpp>

namespace driver {
namespace velodyne {

////////////////////////////////////////////////////////////////////////
// InputPCAP class implementation
////////////////////////////////////////////////////////////////////////

BagInput::BagInput(const std::string packet_topic, bool use_simulation_time)
    : topic_name_(packet_topic),
      use_bag_time_(use_simulation_time),
      bag_packets_() {}

/** @brief destructor */
BagInput::~BagInput() { bag_packets_->Stop(); }

// this callback function run in main thread
void BagInput::PacketCallback(
    const velodyne_msgs::VelodyneScan::ConstPtr& msg) {
  for (auto& msg_packet : msg->packets) {
    auto& data = msg_packet.data;
    TimeHDLDataPacket new_packet;
    HDLDataPacket* packet = &new_packet.packet;
    new_packet.nsec = msg_packet.stamp.toNSec();
    memcpy(packet, &data[0], 1206);
    bag_packets_->Enqueue(new_packet);
  }
}

int BagInput::Init() {
  // create packets queue
  bag_packets_.reset(new SynchronizedQueue<TimeHDLDataPacket>());

  if (topic_name_.empty()) {
    MY_LOG_ERROR("bag input packets topic is empty");
    return static_cast<int>(BAG::BagErrorCode::PACKETS_TOPIC_EMPTY);
  }

  ros::NodeHandle node;
  sub_packets_ =
      node.subscribe(topic_name_, 1, &BagInput::PacketCallback, this);

  return static_cast<int>(BAG::BagErrorCode::OK);
}

/** @brief Get one velodyne packet. */
int BagInput::ReceivePacket(void* data, const int length, uint64_t* nsec) {
  TimeHDLDataPacket pop_time_packet;
  bag_packets_->Dequeue(pop_time_packet);
  HDLDataPacket* packet = &pop_time_packet.packet;
  memcpy(data, packet, length);
  *nsec = pop_time_packet.nsec;
  return static_cast<int>(PCAP::PCAPErrorCode::OK);
}

}  // namespace velodyne
}  // namespace driver
