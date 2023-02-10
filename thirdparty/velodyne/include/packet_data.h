#ifndef DRIVER_VELODYNE_PACKET_DATA_H
#define DRIVER_VELODYNE_PACKET_DATA_H

#include <stdint.h>
#include <string>
#include <unordered_map>

namespace driver {
namespace velodyne {

enum SensorType {
  UNKNOWN = 0x00,     // decimal: 00
  HDL32E = 0x21,      // decimal: 33
  VLP16 = 0x22,       // decimal: 34
  VLP32AB = 0x23,     // decimal: 35
  VLP16HiRes = 0x24,  // decimal: 36
  VLP32C = 0x28,      // decimal: 40
  VLP64 = 0x30,       // decimal: 48
};

enum StatusType {
  HOURS = 72,
  MINUTES = 77,
  SECONDS = 83,
  DATE = 68,
  MONTH = 78,
  YEAR = 89,
  GPS_STATUS = 71
};

enum HDLBlock { BLOCK_0_TO_31 = 0xeeff, BLOCK_32_TO_63 = 0xddff };

enum ReturnModel {
  STRONGEST_RETURN = 0x37,
  LAST_RETURN = 0x38,
  DUAL_RETURN = 0x39,
};

#pragma pack(push, 1)
struct HDLLaserReturn {
  uint16_t distance;
  uint8_t intensity;
};

struct HDLFiringData {
  uint16_t block_identifier;
  uint16_t azimuth;
  HDLLaserReturn laser_returns[32];
};

struct HDLDataPacket {
  HDLFiringData firing_data[12];
  uint8_t gps_timestamp[4];
  uint8_t return_mode;
  uint8_t product_type;
  unsigned int get_gps_time_stamp() {
    return (*reinterpret_cast<unsigned int*>(gps_timestamp));
  }
  unsigned int get_gps_time_stamp() const {
    return (*reinterpret_cast<unsigned int const*>(gps_timestamp));
  }
  std::string get_product_type(
      const std::unordered_map<uint8_t, std::string> sensor_map) {
    if (sensor_map.count(product_type) > 0) {
      return sensor_map.at(product_type);
    } else {
      return "Unknown Type";
    }
  }
  std::string get_return_mode(
      const std::unordered_map<uint8_t, std::string> mode_map) {
    if (mode_map.count(return_mode) > 0) {
      return mode_map.at(return_mode);
    } else {
      return "Unknown Return Mode";
    }
  }
  static const unsigned int GetValidDataLength() { return 1206; }
  static const unsigned int GetRawDataLength() { return 1248; }
  static inline bool IsValidPacket(const unsigned char* data,
                                   unsigned int dataLength) {
    if (dataLength != GetValidDataLength()) return false;
    const HDLDataPacket* dataPacket =
        reinterpret_cast<const HDLDataPacket*>(data);
    return (dataPacket->firing_data[0].block_identifier ==
            HDLBlock::BLOCK_0_TO_31);
  }
  inline bool IsDualModeReturn() const {
    return firing_data[1].azimuth == firing_data[0].azimuth;
  }
};
struct TimeHDLDataPacket {
  uint64_t nsec;
  HDLDataPacket packet;
};

// 64
struct HDL64RawBlock {
  uint16_t laser_block_id;  ///< UPPER_BANK or LOWER_BANK
  uint16_t rotation;        ///< 0-35999, divide by 100 to get degrees
  uint8_t data[96];
};

struct HDL64RawPacket {
  HDL64RawBlock blocks[12];
  unsigned int gps_timestamp;
  unsigned char status_type;
  unsigned char status_value;
};

union RawDistance {
  uint16_t raw_distance;
  uint8_t bytes[2];
};

#pragma pack(pop)

}  // namespace velodyne
}  // namespace driver
#endif  // DRIVER_VELODYNE_PACKET_DATA_H