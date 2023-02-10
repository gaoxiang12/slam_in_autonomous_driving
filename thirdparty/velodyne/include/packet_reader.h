#ifndef DRIVER_VELODYNE_PACKET_READER_H
#define DRIVER_VELODYNE_PACKET_READER_H

#include "velodyne_msgs/VelodynePacket.h"
#include "velodyne_msgs/VelodyneScan.h"

#include "input.h"
#include "packet_data.h"
#include "packet_process.h"
// #include "pcap_input.h"
#include "bag_input.h"
#include "custom_log.h"
#include "diagnostic_constant.h"
#include "socket_input.h"
#include "synchronized_queue.h"
#include "velodyne_config.h"
#include "velodyne_constant.h"

namespace driver {
namespace velodyne {
class PacketConsumer;
}
}  // namespace driver

namespace driver {
namespace velodyne {

class PacketReceiver {
 public:
  PacketReceiver(PacketConsumer* consumer)
      : is_reading_(false),
        thread_(),
        consumer_handler_(consumer),
        current_packet_angle_(0) {
    if (Config::input_type == "socket") {
      input_.reset(new SocketInput(Config::ip_addr,
                                   static_cast<uint16_t>(Config::port)));
      // } else if (Config::input_type == "pcap") {
      //   input_.reset(
      //       new PCAPInput(Config::pcap_file, SensorType::VLP16,
      //       Config::speed));
    } else if (Config::input_type == "bag") {
      input_.reset(new BagInput(Config::packets_topic, false));
    } else {
      input_.reset(new SocketInput(Config::ip_addr,
                                   static_cast<uint16_t>(Config::port)));
    }
  }

  // TODO: safe release thread and ptr
  ~PacketReceiver() {
    is_reading_ = false;
    thread_->join();
    consumer_handler_ = nullptr;
  }

  void LoopReader() {
    while (is_reading_) {
      TimeHDLDataPacket* timed_packet = new TimeHDLDataPacket();
      HDLDataPacket* new_packet = &timed_packet->packet;
      auto result = input_->ReceivePacket(reinterpret_cast<void*>(new_packet),
                                          static_cast<int>(kPacketDataSize),
                                          &timed_packet->nsec);
      if (result != static_cast<int>(ErrorCode::OK)) {
        // ERROR
        MY_LOG_ERROR("packet reader input receive failed "
                     << static_cast<int>(result));
        delete timed_packet;
        continue;
      }

      if (!HDLDataPacket::IsValidPacket(
              reinterpret_cast<unsigned char*>(new_packet),
              static_cast<int>(kPacketDataSize))) {
        MY_LOG_ERROR("packet reader received packet data is not valided");
        delete timed_packet;
        continue;
      }
      std::string packet_lidar_type =
          new_packet->get_product_type(SensorTypeName);
      if (packet_lidar_type != Config::lidar_type) {
        MY_LOG_ERROR("packet reader lidar type is not correct: "
                     << packet_lidar_type << " " << Config::lidar_type);
        delete timed_packet;
        continue;
      }
      consumer_handler_->Enqueue(timed_packet);
    }
  }

  bool Start() {
    if (input_->Init() != static_cast<int>(ErrorCode::OK)) {
      MY_LOG_ERROR("init packet input failed");
      return false;
    }

    if (!thread_) {
      is_reading_ = true;
      thread_.reset(
          new boost::thread(boost::bind(&PacketReceiver::LoopReader, this)));
    } else {
      MY_LOG_ERROR("thread is not empty");
      return false;
    }

    return true;
  }

 private:
  bool is_reading_;
  boost::shared_ptr<boost::thread> thread_;
  PacketConsumer* consumer_handler_;
  boost::shared_ptr<Input> input_;
  // fake packet
  uint16_t current_packet_angle_;
};
}  // namespace velodyne
}  // namespace driver
#endif  // DRIVER_VELODYNE_PACKET_READER_H