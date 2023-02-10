#ifndef DRIVER_VELODYNE_VELODYNE_LIDAR_MODEL_H
#define DRIVER_VELODYNE_VELODYNE_LIDAR_MODEL_H

#include <unistd.h>
#include <array>
#include <cmath>
#include <map>
#include <vector>

#include "packet_data.h"
#include "velodyne_constant.h"

namespace driver {
namespace velodyne {

class VelodyneBase {
 public:
  VelodyneBase() {
    // init the azimuth angle
    for (int rot_step = 0; rot_step < kRotationMaxUnits; rot_step++) {
      // CHECK_LT(rot_step, rot_table_sin_.size());
      float rot_angle = rot_step * 2.0 * M_PI / kRotationMaxUnits;
      rot_table_cos_[rot_step] = cosf(rot_angle);
      rot_table_sin_[rot_step] = sinf(rot_angle);
    }
  }
  const int& get_ring_count() { return ring_count_; }
  const float& get_firing_time() { return firing_recharge_time_; }
  // TODO: calculate firing azimuth diff according to rpm
  const float& get_firing_azimuth_diff(float rpm = 600) {
    return firing_azimuth_diff_;
  }
  const std::array<float, kLaserPerFiring>& get_azimuth_corrected_table() {
    return azimuth_corrected_table_;
  }
  const std::array<int, kLaserPerFiring>& get_azimuth_offset_table() {
    return azimuth_offsets_;
  }
  const std::array<float, kLaserPerFiring>& get_laser_vetical_angles() {
    return laser_vertical_angles_;
  }
  const std::array<int, kLaserPerFiring>& get_firing_laser_sequence() {
    return firing_laser_sequence_;
  }
  const std::array<float, kLaserPerFiring>& get_vert_angle_table_cos() {
    return vert_angle_table_cos_;
  }
  const std::array<float, kLaserPerFiring>& get_vert_angle_table_sin() {
    return vert_angle_table_sin_;
  }
  const std::array<float, kRotationMaxUnits>& get_rot_table_sin() {
    return rot_table_sin_;
  }
  const std::array<float, kRotationMaxUnits>& get_rot_table_cos() {
    return rot_table_cos_;
  }
  const std::array<int, kLaserPerFiring64>& get_64_max_intensity() {
    return max_intensity_;
  }
  const std::array<int, kLaserPerFiring64>& get_64_min_intensity() {
    return min_intensity_;
  }
  const std::array<int, kLaserPerFiring64>& get_64_offsets() {
    return offsets_;
  }
  const std::array<float, kLaserPerFiring64>& get_64_focal_distance() {
    return focal_distance_;
  }
  const std::array<float, kLaserPerFiring64>& get_64_focal_slope() {
    return focal_slope_;
  }
  const std::array<float, kLaserPerFiring64>& get_64_rot_correction() {
    return rot_correction_;
  }
  const std::array<float, kLaserPerFiring64>& get_64_vert_correction() {
    return vert_correction_;
  }
  const std::array<float, kLaserPerFiring64>& get_64_cos_rot_correction() {
    return cos_rot_correction_;
  }
  const std::array<float, kLaserPerFiring64>& get_64_sin_rot_correction() {
    return sin_rot_correction_;
  }
  const std::array<float, kLaserPerFiring64>& get_64_cos_vert_correction() {
    return cos_vert_correction_;
  }
  const std::array<float, kLaserPerFiring64>& get_64_sin_vert_correction() {
    return sin_vert_correction_;
  }
  const std::array<float, kLaserPerFiring64>& get_64_focal_offset() {
    return focal_offset_;
  }
  const std::array<float, kLaserPerFiring64>& get_64_dist_correction() {
    return dist_correction_;
  }
  const std::array<int, kLaserPerFiring64>& get_64_laser_ring() {
    return laser_ring_;
  }
  const std::array<float, kLaserPerFiring64>& get_64_dist_correction_x() {
    return dist_correction_x_;
  }
  const std::array<float, kLaserPerFiring64>& get_64_dist_correction_y() {
    return dist_correction_y_;
  }
  const std::array<float, kLaserPerFiring64>& get_64_vert_offset_correction() {
    return vert_offset_correction_;
  }
  const std::array<float, kLaserPerFiring64>& get_64_horiz_offset_correction() {
    return horiz_offset_correction_;
  }
  const std::array<double, 4>& get_previous_packet_stamp() {
    return previous_packet_stamp_;
  }
  const std::array<uint64_t, 4>& get_gps_base_usec() {
    return gps_base_usec_;
  }

  const float get_distance_resolution() { return distance_resolution_; }
  static int LaserNumber(SensorType sensorType) {
    switch (sensorType) {
      case HDL32E:
      case VLP32AB:
      case VLP32C:
        return 32;
      case VLP16:
      case VLP16HiRes:
        return 16;
      case VLP64:
        return 64;
      default:
        return 0;
    }
  }
  void set_base_time_from_packets (
  const velodyne_msgs::VelodynePacket& pkt) {
  const HDL64RawPacket* raw = (const HDL64RawPacket*)&pkt.data[0];
  StatusType status_type = StatusType(raw->status_type);
  char status_value = raw->status_value;

  static int year = -1, month = -1, day = -1, hour = -1, minute = -1,
             second = -1;
  static int gps_status = 0;
  static tm time;

  switch (status_type) {
  case YEAR:
    year = status_value + 2000;
    break;
  case MONTH:
    month = status_value;
    break;
  case DATE:
    day = status_value;
    break;
  case HOURS:
    hour = status_value;
    break;
  case MINUTES:
    minute = status_value;
    break;
  case SECONDS:
    second = status_value;
    break;
  case GPS_STATUS:
    gps_status = status_value;
    break;
  default:
    break;
  }

  ROS_INFO("Get base time from packets. Obtained (%d.%d.%d %d:%d:%d)", year,
           month, day, hour, minute, second);

  if (status_type == GPS_STATUS && year > 0 && month > 0 && day > 0 &&
      hour >= 0 && minute >= 0 && second >= 0) {
    time.tm_year = year - 1900;
    time.tm_mon = month - 1;
    time.tm_mday = day;
    time.tm_hour = hour;
    time.tm_min = 0;
    time.tm_sec = 0;

    ROS_INFO("Set base unix time: (%d.%d.%d %d:%d:%d)", time.tm_year,
             time.tm_mon, time.tm_mday, time.tm_hour, time.tm_min, time.tm_sec);
    uint64_t unix_base = static_cast<uint64_t>(timegm(&time));
    for (int i = 0; i < 4; ++i) {
      gps_base_usec_[i] = unix_base * 1000000;
    }
  }
}

 protected:
  // custom params
  std::array<float, kRotationMaxUnits> rot_table_sin_;
  std::array<float, kRotationMaxUnits> rot_table_cos_;
  // VLP16 & VLP32 lidar params
  std::array<std::array<float, kLaserPerFiring>, kFiringPerPacket>
      single_return_timing_offsets_;  // in us
  std::array<float, kLaserPerFiring> azimuth_corrected_table_;
  std::array<float, kLaserPerFiring> laser_vertical_angles_;
  std::array<float, kLaserPerFiring> vert_angle_table_cos_;
  std::array<float, kLaserPerFiring> vert_angle_table_sin_;
  std::array<int, kLaserPerFiring> firing_laser_sequence_;
  // azimuth offset of laser rings
  std::array<int, kLaserPerFiring> azimuth_offsets_;
  int ring_count_;
  float distance_resolution_;
  float firing_recharge_time_;  // 1*e-6 seconds
  float firing_azimuth_diff_;   // 0.01 degree
  float max_angle_ = 36000.0;
  float min_angle_ = 0.0;

  bool organized_ = false;
  // 64
  std::array<int, kLaserPerFiring64> max_intensity_;
  std::array<int, kLaserPerFiring64> min_intensity_;
  std::array<int, kLaserPerFiring64> offsets_;
  std::array<int, kLaserPerFiring64> laser_ring_;
  std::array<float, kLaserPerFiring64> focal_distance_;
  std::array<float, kLaserPerFiring64> focal_slope_;
  std::array<float, kLaserPerFiring64> rot_correction_;
  std::array<float, kLaserPerFiring64> vert_correction_;
  std::array<float, kLaserPerFiring64> cos_rot_correction_;
  std::array<float, kLaserPerFiring64> sin_rot_correction_;
  std::array<float, kLaserPerFiring64> cos_vert_correction_;
  std::array<float, kLaserPerFiring64> sin_vert_correction_;
  std::array<float, kLaserPerFiring64> focal_offset_;
  std::array<float, kLaserPerFiring64> dist_correction_;
  std::array<float, kLaserPerFiring64> dist_correction_x_;
  std::array<float, kLaserPerFiring64> dist_correction_y_;
  std::array<float, kLaserPerFiring64> vert_offset_correction_;
  std::array<float, kLaserPerFiring64> horiz_offset_correction_;

  std::array<double, 4> previous_packet_stamp_;
  std::array<uint64_t, 4> gps_base_usec_;
};

class VelodyneVLP16 : public VelodyneBase {
 public:
  VelodyneVLP16() : VelodyneBase() {
    ring_count_ = 16;
    firing_recharge_time_ = 110.592;
    firing_azimuth_diff_ = 39.81;
    // init params
    for (size_t firing = 0; firing < kFiringPerPacket; firing++) {
      for (size_t dsr = 0; dsr < kLaserPerFiring; dsr++) {
        single_return_timing_offsets_[firing][dsr] =
            firing * firing_recharge_time_ + (dsr % 16) * 2.304 + floor(dsr / 16.0) * 55.296;
      }
    }
    for (size_t dsr = 0; dsr < kLaserPerFiring; dsr++) {
      azimuth_corrected_table_[dsr] =
          single_return_timing_offsets_[0][dsr] / firing_recharge_time_;
    }

    azimuth_offsets_ = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                        0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

    laser_vertical_angles_ = {-15.0, 1.0, -13.0, 3.0,  -11.0, 5.0,  -9.0, 7.0,
                              -7.0,  9.0, -5.0,  11.0, -3.0,  13.0, -1.0, 15.0,
                              -15.0, 1.0, -13.0, 3.0,  -11.0, 5.0,  -9.0, 7.0,
                              -7.0,  9.0, -5.0,  11.0, -3.0,  13.0, -1.0, 15.0};
    // firing_laser_sequence_ = {0,  8,  1,  9,  2,  10, 3, 11, 4, 12, 5,
    //                           13, 6,  14, 7,  15, 0,  8, 1,  9, 2,  10,
    //                           3,  11, 4,  12, 5,  13, 6, 14, 7, 15};
    firing_laser_sequence_ = {
        0,      8,       1,      9,       2,      10,      3,      11,
        4,      12,      5,      13,      6,      14,      7,      15,
        0 + 16, 8 + 16,  1 + 16, 9 + 16,  2 + 16, 10 + 16, 3 + 16, 11 + 16,
        4 + 16, 12 + 16, 5 + 16, 13 + 16, 6 + 16, 14 + 16, 7 + 16, 15 + 16};
    for (size_t dsr = 0; dsr < kLaserPerFiring; dsr++) {
      float angle = laser_vertical_angles_[dsr] / 180.0 * M_PI;
      vert_angle_table_cos_[dsr] = cosf(angle);
      vert_angle_table_sin_[dsr] = sinf(angle);
    }

    distance_resolution_ = kLaserDistanceResolutionVLP16;
  }
};

class VelodyneVLP16HD : public VelodyneBase {
 public:
  VelodyneVLP16HD() : VelodyneBase() {
    ring_count_ = 16;
    firing_recharge_time_ = 110.592;
    firing_azimuth_diff_ = 39.81;
    // init params
    for (size_t firing = 0; firing < kFiringPerPacket; firing++) {
      for (size_t dsr = 0; dsr < kLaserPerFiring; dsr++) {
        single_return_timing_offsets_[firing][dsr] =
            firing * firing_recharge_time_ + (dsr % 16) * 2.304 + floor(dsr / 16.0) * 55.296;
      }
    }
    for (size_t dsr = 0; dsr < kLaserPerFiring; dsr++) {
      azimuth_corrected_table_[dsr] =
          single_return_timing_offsets_[0][dsr] / firing_recharge_time_;
    }
    azimuth_offsets_ = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                        0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

    laser_vertical_angles_ = {-10.0, 0.67,  -8.67, 2.0,   -7.33, 3.33,  -6.00,
                              4.67,  -4.67, 6.00,  -3.33, 7.33,  -2.00, 8.67,
                              -0.67, 10.0,  -10.0, 0.67,  -8.67, 2.0,   -7.33,
                              3.33,  -6.00, 4.67,  -4.67, 6.00,  -3.33, 7.33,
                              -2.00, 8.67,  -0.67, 10.0};
    // firing_laser_sequence_ = {0,  8,  1,  9,  2,  10, 3, 11, 4, 12, 5,
    //                           13, 6,  14, 7,  15, 0,  8, 1,  9, 2,  10,
    //                           3,  11, 4,  12, 5,  13, 6, 14, 7, 15};
    firing_laser_sequence_ = {
        0,      8,       1,      9,       2,      10,      3,      11,
        4,      12,      5,      13,      6,      14,      7,      15,
        0 + 16, 8 + 16,  1 + 16, 9 + 16,  2 + 16, 10 + 16, 3 + 16, 11 + 16,
        4 + 16, 12 + 16, 5 + 16, 13 + 16, 6 + 16, 14 + 16, 7 + 16, 15 + 16};

    distance_resolution_ = kLaserDistanceResolutionVLP16;
  }
};

class VelodyneVLP32 : public VelodyneBase {
 public:
  VelodyneVLP32() : VelodyneBase() {
    ring_count_ = 32;
    firing_recharge_time_ = 55.296;
    firing_azimuth_diff_ = 19.91;
    // init params
    for (size_t firing = 0; firing < kFiringPerPacket; firing++) {
      for (size_t dsr = 0; dsr < kLaserPerFiring; dsr++) {
        int firing_index = trunc(static_cast<float>(dsr) / 2.0);
        single_return_timing_offsets_[firing][dsr] =
            firing * firing_recharge_time_ + firing_index * 2.304;
      }
    }

    for (size_t dsr = 0; dsr < kLaserPerFiring; dsr++) {
      azimuth_corrected_table_[dsr] =
          single_return_timing_offsets_[0][dsr] / firing_recharge_time_;
    }

    std::array<float, 32> azimuth_offsets_degrees = {
        1.4,  -4.2, 1.4,  -1.4, 1.4,  -1.4, 4.2,  -1.4, 1.4,  -4.2, 1.4,
        -1.4, 4.2,  -1.4, 4.2,  -1.4, 1.4,  -4.2, 1.4,  -4.2, 4.2,  -1.4,
        1.4,  -1.4, 1.4,  -1.4, 1.4,  -4.2, 4.2,  -1.4, 1.4,  -1.4};
    for (size_t idx = 0; idx < azimuth_offsets_.size(); ++idx) {
      azimuth_offsets_[idx] =
          static_cast<int>(azimuth_offsets_degrees[idx] / kRotationResolution);
    }

    laser_vertical_angles_ = {
        -25.0,  -1.0,  -1.667, -15.639, -11.31, 0.0,   -0.667, -8.843,
        -7.254, 0.333, -0.333, -6.148,  -5.333, 1.333, 0.667,  -4.0,
        -4.667, 1.667, 1.0,    -3.667,  -3.333, 3.333, 2.333,  -2.667,
        -3.0,   7.0,   4.667,  -2.333,  -2.0,   15.0,  10.333, -1.333};

    firing_laser_sequence_ = {0,  17, 15, 1,  2,  20, 18, 3,  4,  21, 19,
                              5,  6,  24, 22, 8,  7,  25, 23, 9,  10, 27,
                              26, 12, 11, 29, 28, 13, 14, 31, 30, 16};

    for (size_t dsr = 0; dsr < kLaserPerFiring; dsr++) {
      float angle = laser_vertical_angles_[dsr] / 180.0 * M_PI;
      vert_angle_table_cos_[dsr] = cosf(angle);
      vert_angle_table_sin_[dsr] = sinf(angle);
    }

    distance_resolution_ = kLaserDistanceResolutionVLP32;
  }
};

class VelodyneVLPHDL32 : public VelodyneBase {
 public:
  VelodyneVLPHDL32() : VelodyneBase() {
    ring_count_ = 32;
    firing_recharge_time_ = 46.08;
    firing_azimuth_diff_ = 16.59;
    // firing_azimuth_diff_ = 16.59 * 2;
    // init params
    for (size_t firing = 0; firing < kFiringPerPacket; firing++) {
      for (size_t dsr = 0; dsr < kLaserPerFiring; dsr++) {
        int firing_index = trunc(static_cast<float>(dsr) / 2.0);
        single_return_timing_offsets_[firing][dsr] =
            firing * firing_recharge_time_ + firing_index * 1.152;
      }
    }

    for (size_t dsr = 0; dsr < kLaserPerFiring; dsr++) {
      azimuth_corrected_table_[dsr] =
          single_return_timing_offsets_[0][dsr] / firing_recharge_time_;
    }

    std::array<float, 32> azimuth_offsets_degrees = {
        0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
    for (size_t idx = 0; idx < azimuth_offsets_.size(); ++idx) {
      azimuth_offsets_[idx] =
          static_cast<int>(azimuth_offsets_degrees[idx] / kRotationResolution);
    }

    laser_vertical_angles_ = {
        -30.67, -9.33, -29.33, -8.0,  -28.00, -6.67, -26.67, -5.33,
        -25.33, -4.00, -24.00, -2.67, -22.67, -1.33, -21.33, 0.00,
        -20.00, 1.33,  -18.67, 2.67,  -17.33, 4.00,  -16.00, 5.33,
        -14.67, 6.67,  -13.33, 8.00,  -12.00, 9.33,  -10.67, 10.67};

    std::map<float, int> angles_map;
    for (size_t idx = 0; idx < laser_vertical_angles_.size(); ++idx) {
      angles_map[laser_vertical_angles_[idx]] = static_cast<int>(idx);
    }
    int count = 0;
    for (auto& angles_map_ele : angles_map) {
      firing_laser_sequence_[angles_map_ele.second] = count;
      count++;
    }

    // firing_laser_sequence_ = {0,  17, 15, 1,  2,  20, 18, 3,  4,  21, 19,
    //                           5,  6,  24, 22, 8,  7,  25, 23, 9,  10, 27,
    //                           26, 12, 11, 29, 28, 13, 14, 31, 30, 16};

    for (size_t dsr = 0; dsr < kLaserPerFiring; dsr++) {
      float angle = laser_vertical_angles_[dsr] / 180.0 * M_PI;
      vert_angle_table_cos_[dsr] = cosf(angle);
      vert_angle_table_sin_[dsr] = sinf(angle);
    }

    distance_resolution_ = kLaserDistanceResolutionVLPHDL32;
  }
};

// 64E_S3
class VelodyneVLPHDL64 : public VelodyneBase {
 public:
  VelodyneVLPHDL64() : VelodyneBase() {
    ring_count_ = 64;
    max_intensity_ = {255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255,
                      255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255,
                      255, 250, 250, 255, 255, 255, 255, 255, 255, 255, 255, 255,
                      255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255,
                      255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255,
                      255, 255, 255, 255};

    min_intensity_ = {40, 40, 40, 25, 40, 20, 40, 40, 10, 30, 40, 25,
                      20, 10, 30, 40, 40, 40, 10, 15, 20, 15, 20, 10,
                      10, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                      0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                      0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                      0, 0, 0, 0};

    focal_distance_ = {12.6, 22.0, 24.0, 15.0, 22.0, 21.5, 12.5, 18.0, 19.5, 16.2, 9.5, 13.6,
                       16.5, 21.0, 12.5, 19.5, 24.0, 15.2, 24.0, 24.0, 24.0, 17.0, 24.0, 19.5,
                       13.5, 24.0, 16.5, 24.0, 24.0, 22.0, 24.0, 24.0, 11.5, 0.25, 0.25, 11.65,
                       8.0, 2.0, 0.25, 0.25, 0.25, 11.5, 0.25, 0.5, 12.0, 0.25, 0.5, 0.25,
                       8.0, 10.0, 7.3, 8.2, 0.25, 11.5, 0.25, 11.5, 8.0, 0.25, 11.5, 0.25,
                       0.25, 0.25, 7.5, 5.5};

    focal_slope_ = {1.6, 0.65, 0.60, 1.35, 0.80, 0.90, 1.25, 0.75, 0.95, 1.55, 1.40, 1.80,
                    1.45, 1.15, 2.0, 1.0, 1.40, 1.65, 1.40, 1.0, 1.15, 1.42, 1.25, 1.20,
                    1.90, 0.65, 1.55, 0.95, 0.70, 0.60, 1.30, 0.85, 2.0, 1.10, 0.95, 1.96,
                    1.30, 0.85, 0.95, 1.10, 1.10, 2.0, 1.10, 1.55, 2.0, 1.05, 1.55, 0.95,
                    1.45, 1.40, 1.30, 1.35, 0.95, 2.0, 0.95, 1.95, 1.60, 1.0, 2.0, 1.0,
                    0.95, 0.95, 1.4, 1.0};

    rot_correction_ = {-0.08196294456431531, -0.04620889047151336, 0.05444975575011892, 0.09280559659347837,
                       -0.006882723533065084, 0.03217355127490839, -0.0213722426050666, 0.018393991825512452,
                       0.06756736417411098, 0.10709353031140746, 0.05393670225386117, 0.09363562806183705,
                       -0.08237526812812347, -0.043652471809532224, -0.09622369652447386, -0.0573145688549397,
                       -0.007995444235040053, 0.03118716274018053, -0.0214217314160069, 0.01764926856033273, 
                       0.0666366272079122, 0.10544232841130242, 0.0532484246820202, 0.09185751105451477,
                       -0.08255157779850143, -0.043864106943970554, -0.09671153873968655, -0.05743587447394106,
                       -0.007871438591685856, .030575405638734746, -0.021957695849355578, 0.01770540707572313,
                       -0.13517721315287398, -0.07499324272251003, 0.08432742238713763, 0.13966302523509402,
                       -0.012244650931746405, 0.04896387698782314, -0.036152727436530266, 0.023467524334719807,
                       0.106910021157865, 0.16722507803235853, 0.08521933974011354, 0.14823788719133674,
                       -0.13064500498314818, -0.07026578282403437, -0.15524346602124603, -0.09494867591002668,
                       -0.010909463581995231, 0.04600423141809525, -0.034298522800467274, 0.025046651118034474,
                       0.10527884342026814, 0.1623772759366286, 0.08441971016199534, 0.1436936868643205,
                       -0.12769945643778866, -0.07124141489458219, -0.1514034536875319, -0.09181169616164991,
                       -0.010857456086411378, 0.04411035370280518, -0.03305086720336212, 0.023995402657643997};
  
  vert_correction_ = {-0.1259806614264124, -0.11994046085769547, 0.004378839107036053, 0.010306535403103584,
                      -0.11490022923061616, -0.10823816610408439, -0.15064448679032666, -0.14364376870238418,
                      -0.1029824334382736, -0.09589844220105996, -0.13923599552700858, -0.13120803772786604,
                      -0.054438932963427306, -0.04811872747748367, -0.08981872737807786, -0.0839353306800186,
                      -0.04322299165917697, -0.03648801216715539, -0.0788587386086325, -0.07215144178387681,
                      -0.031996161245723444, -0.02505170559154825, -0.06747277081931138, -0.060754789088874966,
                      0.01602915673379226, 0.02032046705524202, -0.01933091598390954, -0.012791343729474167,
                      0.027675144896805965, 0.034414332377654115, -0.007885903531460533, -0.0015491891506552067,
                      0.3905617864033169, -0.3839017670573393, -0.19556576584216898, -0.18669910926960234,
                      -0.3775589787562041, -0.3667384958991999, -0.42789019031327086, -0.4209303583083856,
                      -0.35816015026577014, -0.3460936594024747, -0.4113537192494002, -0.39996352601796,
                      -0.28478490280901675, -0.275534430880644, -0.33538020013520276, -0.32788765873602377,
                      -0.2702265530127714, -0.25937308343290444, -0.3214611818972554, -0.3116772675363032,
                      -0.24999258447525818, -0.23805884335486943, -0.30499605989320633, -0.2904239045425777,
                      -0.17562358144900422, -0.16828168639888186, -0.2289658175783191, -0.22158565792967605,
                      -0.1609214427023333, -0.15234551434991764, -0.21535165600081768, -0.2038133366757607};

  dist_correction_ = {1.2367596, 1.1936214, 1.459742, 1.3109583, 1.2347497, 1.1445649,
                      1.416552, 1.3378871, 1.3826251, 1.3542729, 1.366015, 1.2997078,
                      1.2170806, 1.1838578, 1.2790835, 1.3350552, 1.24064, 1.3757257,
                      1.399003, 1.2935248, 1.2455103, 1.2038242, 1.2699283, 1.1648331,
                      1.2876584, 1.3608078, 1.4493187, 1.3432137, 1.2902887, 1.2217331,
                      1.290667, 1.4064978, 1.1718893, 1.1005492, 1.2321508, 1.2242654,
                      1.2047442, 1.1727358, 1.3291516, 1.1898002, 1.1466835, 1.1651333,
                      1.3327728, 1.2008248, 1.184101, 1.1944227, 1.3340642, 1.206831,
                      1.1827332, 1.2426594, 1.1906987, 1.0193738, 1.1831448, 1.3809126,
                      1.3415497, 1.1950829, 1.2653324, 1.2114046, 1.4149663, 1.249181,
                      1.2333247, 1.2433365, 1.3713475, 1.2561033};

  dist_correction_x_ = {1.2553750, 1.2343914, 1.4864447, 1.3262161, 1.2844521, 1.166612,
                        1.4560255, 1.3709512, 1.4276646, 1.3835085, 1.4010266, 1.3284657,
                        1.2562402, 1.2000351, 1.3382445, 1.3506117, 1.2572079, 1.4149931,
                        1.4414699, 1.3158357, 1.2845569, 1.2312194, 1.3184879, 1.1831214,
                        1.3071066, 1.3717859, 1.4723048, 1.3748969, 1.3032738, 1.2268237,
                        1.3211456, 1.4215631, 1.2078307, 1.1137292, 1.2281995, 1.2363837,
                        1.2474401, 1.1935702, 1.3434761, 1.1855456, 1.1833476, 1.1731619,
                        1.3462103, 1.2033921, 1.2344692, 1.2371402, 1.3850056, 1.2474676,
                        1.2390784, 1.2799032, 1.2505486, 1.0610925, 1.2116116, 1.3977461, 
                        1.3832455, 1.2248234, 1.2948512, 1.2215437, 1.4521725, 1.2679234,
                        1.2620737, 1.2394186, 1.3939098, 1.2643785};

  dist_correction_y_ = {1.2662602, 1.2281053, 1.4992131, 1.3620294, 1.2686826, 1.1872234,
                        1.4212427, 1.3732817, 1.4142607, 1.4049788, 1.3781967, 1.3377197,
                        1.2233616, 1.1696870, 1.2937793, 1.3250481, 1.221573, 1.4001311,
                        1.3909142, 1.3393175, 1.2703239, 1.2405616, 1.2799171, 1.2200997,
                        1.2925192, 1.3802054, 1.4443196, 1.3669669, 1.3025902, 1.2241704,
                        1.2808319, 1.4324644, 1.195512, 1.1310715, 1.244636, 1.2887556,
                        1.2280060, 1.2215664, 1.3512526, 1.2228506, 1.1800524, 1.1876134,
                        1.3560004, 1.2363107, 1.1753256, 1.2367662, 1.3688501, 1.2625677,
                        1.2194650, 1.2577645, 1.2336496, 1.0712325, 1.218315, 1.4523634,
                        1.3744815, 1.2613467, 1.2723473, 1.2463406, 1.4046565, 1.2816116,
                        1.2717489, 1.2621243, 1.3859982, 1.2973643};

  vert_offset_correction_ = {0.21567997, 0.21523903, 0.20625826, 0.2058320, 0.21487156, 0.2143865,
                             0.21748789, 0.21697344, 0.21400434, 0.21348989, 0.21665007, 0.21606213,
                             0.21049141, 0.21003576, 0.21304895, 0.21262268, 0.209683, 0.20919794,
                             0.21225523, 0.21177017, 0.20887459, 0.20837484, 0.21143211, 0.21094707,
                             0.20542046, 0.20511177, 0.20796328, 0.20749292, 0.20458263, 0.2040976,
                             0.20714018, 0.20668451, 0.15973328, 0.1592506, 0.14645961, 0.14588801,
                             0.15879332, 0.1580185, 0.16248962, 0.16196884, 0.15740880, 0.15655776,
                             0.16125753, 0.1604192, 0.15234067, 0.15171827, 0.15580834, 0.15528755,
                             0.15136261, 0.15063859, 0.154843, 0.15416977, 0.15001619, 0.14922866,
                             0.15371249, 0.15272173, 0.1451767, 0.14470672, 0.14863166, 0.14814898,
                             0.14423674, 0.14369055, 0.14774252, 0.14699309};

  horiz_offset_correction_ = {0.025999999, -0.025999999, 0.025999999, -0.025999999, 0.025999999, -0.025999999,
                              0.025999999, -0.025999999, 0.025999999, -0.025999999, 0.025999999, -0.025999999,
                              0.025999999, -0.025999999, 0.025999999, -0.025999999, 0.025999999, -0.025999999,
                              0.025999999, -0.025999999, 0.025999999, -0.025999999, 0.025999999, -0.025999999,
                              0.025999999, -0.025999999, 0.025999999, -0.025999999, 0.025999999, -0.025999999,
                              0.025999999, -0.025999999, 0.025999999, -0.025999999, 0.025999999, -0.025999999,
                              0.025999999, -0.025999999, 0.025999999, -0.025999999, 0.025999999, -0.025999999,
                              0.025999999, -0.025999999, 0.025999999, -0.025999999, 0.025999999, -0.025999999,
                              0.025999999, -0.025999999, 0.025999999, -0.025999999, 0.025999999, -0.025999999,
                              0.025999999, -0.025999999, 0.025999999, -0.025999999, 0.025999999, -0.025999999,
                              0.025999999, -0.025999999, 0.025999999, -0.025999999};

  for (size_t firing = 0; firing < ring_count_; firing++) {
      cos_rot_correction_[firing] = cosf(rot_correction_[firing]);
      sin_rot_correction_[firing] = sinf(rot_correction_[firing]);
      cos_vert_correction_[firing] = cosf(vert_correction_[firing]);
      sin_vert_correction_[firing] = sinf(vert_correction_[firing]);
      focal_offset_[firing] = 256 * pow(1 - focal_distance_[firing] / 13100, 2);
  }
  organized_ = true;
  if (organized_) {
    for (int i = 0; i < ring_count_; ++i) {
      int col = ORDER_64[i];
      int offset = int(rot_correction_[col] / kAngularResolution + 0.5);
      offsets_[i] = offset;
    }
  }
  double next_angle = -std::numeric_limits<double>::infinity();
  for (int ring = 0; ring < ring_count_; ++ring) {
    double min_seen = std::numeric_limits<double>::infinity();
    int next_index = ring_count_;
    for (int j = 0; j < ring_count_; ++j) {
      double angle = vert_correction_[j];
      if (next_angle < angle && angle < min_seen) {
        min_seen = angle;
        next_index = j;
      }
    }
    if (next_index < ring_count_) { 
      laser_ring_[next_index] = ring;
      next_angle = min_seen;
    }
  }

  double view_direction = 0.0;
  double view_width = 2 * M_PI;
  double tmp_min_angle = view_direction + view_width / 2;
  double tmp_max_angle = view_direction - view_width / 2;
  tmp_min_angle = fmod(fmod(tmp_min_angle, 2 * M_PI) + 2 * M_PI, 2 * M_PI);
  tmp_max_angle = fmod(fmod(tmp_max_angle, 2 * M_PI) + 2 * M_PI, 2 * M_PI);
  min_angle_ = 100 * (2 * M_PI - tmp_min_angle) * 180 / M_PI + 0.5;
  max_angle_ = 100 * (2 * M_PI - tmp_max_angle) * 180 / M_PI + 0.5;
  if (min_angle_ == max_angle_) {
    min_angle_ = 0;
    max_angle_ = 36000;
  }

  for (int i = 0; i < 4; i++) {
    gps_base_usec_[i] = 0;
    previous_packet_stamp_[i] = 0;
  }
}

};

}  // namespace velodyne
}  // namespace driver
#endif  // DRIVER_VELODYNE_VELODYNE_LIDAR_MODEL_H
