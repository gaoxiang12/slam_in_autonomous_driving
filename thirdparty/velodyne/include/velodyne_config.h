#ifndef DRIVER_VELODYNE_VELODYNE_CONFIG_H
#define DRIVER_VELODYNE_VELODYNE_CONFIG_H

#include <string>

#include "velodyne_constant.h"

namespace driver {
namespace velodyne {
struct CalibParams {
  static float x_offset;
  static float y_offset;
  static float z_offset;
  static float roll;
  static float pitch;
  static float yaw;
  static float car_left;
  static float car_right;
  static float car_front;
  static float car_back;
  static float car_top;
  static float car_bottom;
};

struct Config {
  static std::string lidar_name;
  static std::string input_type;  // socket, pcap
  static std::string lidar_type;  // HDL32E VLP16 VLP32AB VLP16HD VLP32C
  static std::string pub_topic;   // point cloud publish topic
  static bool enable_pub_packets;
  static std::string
      pub_packets_topic;        // packets publish topic, for bag record
  static std::string frame_id;  // cloud pub frame id
  // for socket input
  static std::string ip_addr;
  static int port;
  // for pcap input
  static std::string pcap_file;
  static double speed;  // pcap play speed
                        // lidar calibration params
  // for bag packets
  static std::string packets_topic;
  static bool enable_coordinate_transformation;
  static CalibParams cali_params;
};

}  // namespace velodyne
}  // namespace driver
#endif  // DRIVER_VELODYNE_VELODYNE_CONFIG_H