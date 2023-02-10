#ifndef DRIVER_VELODYNE_PACKET_PROCESS_H
#define DRIVER_VELODYNE_PACKET_PROCESS_H

#include <functional>

#include <ros/ros.h>

#include <thread>

#include <pcl_conversions/pcl_conversions.h>

#include "velodyne_msgs/VelodynePacket.h"
#include "velodyne_msgs/VelodyneScan.h"

#include "lidar_transform.h"
#include "packet_data.h"
#include "point_types.h"
#include "synchronized_queue.h"
#include "velodyne_config.h"
#include "velodyne_constant.h"
#include "velodyne_lidar_model.h"

namespace driver {
namespace velodyne {

  using FiringCloud = std::vector<driver::velodyne::PointXYZRRIAR>;
  using FiringCloudPtr = std::vector<driver::velodyne::PointXYZRRIAR>*;
  using PointCloudCallbackHandle = std::function<void(
      pcl::PointCloud<driver::velodyne::PointXYZRRIAR>::Ptr&)>;
  using PacketsCallbackHandle =
      std::function<void(velodyne_msgs::VelodyneScan::Ptr&)>;

class PacketConsumer {
  

 public:
  PacketConsumer(bool enable_pub_packets, float phase_lock_angle)
      : is_processing_(false),
        thread_(),
        cloud_(new pcl::PointCloud<driver::velodyne::PointXYZRRIAR>()),
        packets_(new SynchronizedQueue<TimeHDLDataPacket*>()),
        packets_msg_(new velodyne_msgs::VelodyneScan()),
        last_azimuth_(36000),
        start_angle_(18000),
        enable_pub_packets_(enable_pub_packets) {
    // VLP16 754 packet/second
    // VLP32 1507 packet/second

    if (SensorTypeByName.count(Config::lidar_type) > 0) {
      if (SensorTypeByName.at(Config::lidar_type) ==
          static_cast<uint8_t>(SensorType::VLP16)) {
        lidar_type_.reset(new VelodyneVLP16());
      } else if (SensorTypeByName.at(Config::lidar_type) ==
                 static_cast<uint8_t>(SensorType::VLP16HiRes)) {
        lidar_type_.reset(new VelodyneVLP16HD());
      } else if (SensorTypeByName.at(Config::lidar_type) ==
                 static_cast<uint8_t>(SensorType::VLP32C)) {
        lidar_type_.reset(new VelodyneVLP32());
      } else if (SensorTypeByName.at(Config::lidar_type) ==
                 static_cast<uint8_t>(SensorType::HDL32E)) {
        lidar_type_.reset(new VelodyneVLPHDL32());
      } else {
        lidar_type_.reset(new VelodyneVLP16());
      }
    } else {
      lidar_type_.reset(new VelodyneVLP16());
    }

    // TODO: too verbose
    start_angle_ =
        static_cast<uint16_t>(static_cast<int>(fabs(phase_lock_angle) * 100) %
                              (kRotationMaxUnits - 200));
    packets_msg_->packets.reserve(160);

    // set calibration params
    lidar_transform_.reset(new LidarTransform(
        CalibParams::roll, CalibParams::pitch, CalibParams::yaw,
        CalibParams::x_offset, CalibParams::y_offset, CalibParams::z_offset));
  }

  ~PacketConsumer() {
    is_processing_ = false;
    // packets must clear before thread
    packets_->Stop();
    thread_->join();
  }

  void Enqueue(TimeHDLDataPacket* time_packet) {
    packets_->Enqueue(time_packet);
  }

  void Start() {
    if (!thread_) {
      is_processing_ = true;
      thread_.reset(new std::thread(std::bind(&PacketConsumer::Process, this)));
    }
  }

  void PushCallback(PointCloudCallbackHandle callback) {
    callback_list_.push_back(callback);
  }

  void SetPacketBagCallback(PacketsCallbackHandle callback) {
    packet_callback_ = callback;
  }

 private:
  inline bool at_end_scanning_boundary(uint16_t current_azimuth,
                                       uint16_t next_azimuth) {
    if (next_azimuth >= start_angle_ && current_azimuth < start_angle_) {
      return true;
    } else {
      return false;
    }
  }

  inline bool at_end_scanning_boundary(uint16_t current_azimuth) {
    if (current_azimuth >= start_angle_ && last_azimuth_ < start_angle_) {
      return true;
    }
    return false;
  }

  //   inline bool at_start_scanning_boundary(uint16_t current_azimuth,
  //                                          uint16_t prev_azimuth) {
  //     if (current_azimuth >= start_angle_ && prev_azimuth < start_angle_)
  //     {
  //       return true;
  //     } else {
  //       return false;
  //     }
  //   }

  inline void ProcessFiring(FiringCloud& firing_points,
                            const HDLFiringData* firing_data, int block,
                            float azimuth_diff, unsigned int rawtime) {
    uint16_t azimuth = firing_data->azimuth;
    uint16_t azimuth_corrected = 0;

    const std::array<float, kLaserPerFiring>& azimuth_corrected_table =
        lidar_type_->get_azimuth_corrected_table();
    const std::array<int, kLaserPerFiring>& azimuth_offsets =
        lidar_type_->get_azimuth_offset_table();
    const std::array<int, kLaserPerFiring>& firing_sequence =
        lidar_type_->get_firing_laser_sequence();
    const float& distance_resolution = lidar_type_->get_distance_resolution();
    const std::array<float, kLaserPerFiring>& vert_angle_table_cos =
        lidar_type_->get_vert_angle_table_cos();
    const std::array<float, kLaserPerFiring>& vert_angle_table_sin =
        lidar_type_->get_vert_angle_table_sin();
    const std::array<float, kRotationMaxUnits>& rot_table_cos =
        lidar_type_->get_rot_table_cos();
    const std::array<float, kRotationMaxUnits>& rot_table_sin =
        lidar_type_->get_rot_table_sin();
    const int& ring_count = lidar_type_->get_ring_count();

    // loop calculate the every firing points
    for (size_t dsr = 0; dsr < kLaserPerFiring; ++dsr) {
      azimuth_corrected = static_cast<uint16_t>(
          (static_cast<int>(azimuth) +
           static_cast<int>(azimuth_diff * azimuth_corrected_table[dsr]) +
           azimuth_offsets[dsr] + kRotationMaxUnits) %
          kRotationMaxUnits);
      // azimuth_corrected =
      //     static_cast<uint16_t>(round(static_cast<double>(azimuth_corrected)))
      //     % kRotationMaxUnits;  // kRotationMaxUnits=36000
      // azimuth_corrected = (int)round(fmod(azimuth_corrected_f, 36000.0));

      driver::velodyne::PointXYZRRIAR& point =
          firing_points[firing_sequence[dsr]];
      // point ring
      point.ring = static_cast<uint8_t>(firing_sequence[dsr] % ring_count);
      // point angle
      point.angle = azimuth_corrected;

      const HDLLaserReturn& laser_returns = firing_data->laser_returns[dsr];
      // point intensity
      point.intensity = laser_returns.intensity;
      // reset point x,y,z,range,radius
      point.x = point.y = point.z = point.range = point.radius = kLaserHitFree;

      float distance = kLaserHitFree;
      float xy_distance = kLaserHitFree;

      if (laser_returns.distance != 0) {
        // calculate xyz coordinate
        distance = laser_returns.distance * distance_resolution;

        // Compute the distance in the xy plane (w/o accounting for rotation)
        xy_distance = distance * vert_angle_table_cos[dsr];

        // Use standard ROS coordinate system (right-hand rule)
        point.x = xy_distance * rot_table_cos[azimuth_corrected];
        point.y = -xy_distance * rot_table_sin[azimuth_corrected];
        point.z = distance * vert_angle_table_sin[dsr];
        point.radius = xy_distance;
        point.range = distance;
      }

      if (Config::enable_coordinate_transformation) {
        lidar_transform_->TransformCloud<driver::velodyne::PointXYZRRIAR>(
            &point);
        lidar_transform_->IsInCarFootPrint(&point);
      }
    }
  }

  inline void ProcessPacket(const HDLDataPacket* data_packet,
                            const uint64_t nsec) {
    const unsigned int rawtime = data_packet->get_gps_time_stamp();

    float azimuth_diff = lidar_type_->get_firing_azimuth_diff();
 
    uint64_t block_offset_nsec =
        static_cast<uint64_t>(lidar_type_->get_firing_time() * 1000);


    for (size_t firing_block = 0; firing_block < kFiringPerPacket;
         ++firing_block) {
      const HDLFiringData* firing_data =
          &(data_packet->firing_data[firing_block]);

      auto current_azimuth = firing_data->azimuth;

      if (at_end_scanning_boundary(current_azimuth)) {
        // calculate cloud header time stamp
        ros::Time cloud_last_point_stamp;
        const uint64_t cloud_nsec = nsec - firing_block * block_offset_nsec;
        cloud_last_point_stamp.fromNSec(cloud_nsec);
        cloud_->header.stamp = pcl_conversions::toPCL(cloud_last_point_stamp);
        cloud_->height = lidar_type_->get_ring_count();
        cloud_->width = cloud_->points.size() / cloud_->height;
        for (auto& cb : callback_list_) {
          cb(cloud_);
        }
        cloud_->points.clear();
      }

      last_azimuth_ = current_azimuth;

      FiringCloudPtr firing_points = new FiringCloud();
      firing_points->resize(kLaserPerFiring);
      ProcessFiring(*firing_points, firing_data, firing_block, azimuth_diff,
                    rawtime);

      cloud_->points.insert(cloud_->points.end(), firing_points->begin(),
                            firing_points->end());

      delete firing_points;
    }
  }

  void Process() {
    TimeHDLDataPacket* ele = nullptr;

    while (packets_->Dequeue(ele) && is_processing_) {
      HDLDataPacket* packet = &ele->packet;
      ProcessPacket(packet, ele->nsec);
      delete ele;
      ele = nullptr;
    }
  }

 private:
  bool is_processing_;
  std::shared_ptr<std::thread> thread_;
  pcl::PointCloud<driver::velodyne::PointXYZRRIAR>::Ptr cloud_;
  std::shared_ptr<SynchronizedQueue<TimeHDLDataPacket*>> packets_;
  velodyne_msgs::VelodyneScan::Ptr packets_msg_;
  std::vector<PointCloudCallbackHandle> callback_list_;
  PacketsCallbackHandle packet_callback_;

  std::shared_ptr<VelodyneBase> lidar_type_;
  std::shared_ptr<LidarTransform> lidar_transform_;

  // the scanning points is in [start_angle, end_angle)
  uint16_t last_azimuth_;
  uint16_t start_angle_;
  bool enable_pub_packets_;
};  // namespace velodyne
}  // namespace velodyne
}  // namespace driver
#endif  // DRIVER_VELODYNE_PACKET_PROCESS_H