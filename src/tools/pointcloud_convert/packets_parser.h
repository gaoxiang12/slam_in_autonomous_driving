#pragma once

#include <angles/angles.h>
#include <pcl_ros/point_cloud.h>
#include <velodyne/include/velodyne_constant.h>

#include "common/lidar_utils.h"
#include "common/point_types.h"
#include "tools/pointcloud_convert/velodyne_config.h"

namespace sad::tools {

/// packets协议解析
class PacketsParser {
   public:
    static const int kSizeBlock = 100;
    static const int kRawScanSize = 3;
    static const int kScanPerBlock = 32;
    static const int kBlockDataSize = (kScanPerBlock * kRawScanSize);

    constexpr static float kRotationResolution = 0.01f;
    constexpr static const uint16_t kRotationMaxUnits = 36000u;
    constexpr static const float kDistanceResolution = 0.002f;

    constexpr static const uint16_t kUpperBank = 0xeeff;
    constexpr static const uint16_t kLowerBank = 0xddff;
    constexpr static const int kVLP16FiringsPerBlock = 2;
    constexpr static const int kVLP16ScanPerFiring = 16;
    constexpr static const float kVLP16BlockTduration = 110.592f;
    constexpr static const float kVLP16DsrToffset = 2.304f;
    constexpr static const float kVLP16FiringToffset = 55.296f;
    constexpr static const double kVLP16ScanVertAngle[kVLP16ScanPerFiring] = {
        -0.2617993877991494,  0.017453292519943295, -0.22689280275926285,  0.05235987755982989,
        -0.19198621771937624, 0.08726646259971647,  -0.15707963267948966,  0.12217304763960307,
        -0.12217304763960307, 0.15707963267948966,  -0.08726646259971647,  0.19198621771937624,
        -0.05235987755982989, 0.22689280275926285,  -0.017453292519943295, 0.2617993877991494};

    struct RawBlock {
        uint16_t header;
        uint16_t rotation;
        uint8_t data[kBlockDataSize];
    };

    union TwoBytes {
        uint16_t uint;
        uint8_t bytes[2];
    };

    static const int kPacketSize = 1206;
    static const int kBlocksPerPacket = 12;
    static const int kPacketStatusSize = 4;
    static const int kScansPerPacket = (kScanPerBlock * kBlocksPerPacket);

    struct RawPacket {
        RawBlock blocks[kBlocksPerPacket];
        uint16_t revolution;
        uint8_t status[kPacketStatusSize];
    };

    static constexpr float HIT_NAN = std::numeric_limits<float>::max();
    static constexpr float HIT_FREE = std::numeric_limits<float>::min();

    PacketsParser() : organized_raw_pointcloud_(kVLP16ScanPerFiring) {
        inner_time_ = &driver::velodyne::INNER_TIME_16;
        azimuth_corrected_table_ = &driver::velodyne::INNER_AZIMUTH_16;
    }

    int Setup(const VelodyneConfig &conf);
    void PaddingPointCloud(const PacketsMsgPtr &scan_msg, FullCloudPtr &out_pc_msg_ptr);

   private:
    /// 输出的点云rings_pointcloud中，每个点的curvature字段保存该点与msg_stamp的时间差
    void Unpack(const velodyne_msgs::VelodynePacket &pkt, const ros::Time &msg_stamp,
                std::vector<FullCloudPtr> &rings_pointcloud);

    inline bool isScanValid(int rotation, float range);

    inline bool isScanValid(const FullPointType &point);

    inline bool isPointValid(const FullPointType &point);

    inline void FilledNAN(FullPointType &point);

    inline void FilledFree(FullPointType &point);

    void ComputeCoords(const float distance, const int vert_line_index, const uint16_t &rotation, FullPointType &point);

    void ArrangePointcloud(const std::vector<FullCloudPtr> &rings_pointcloud, FullCloudPtr &out_pc_ptr);

    bool PointInRange(float range) { return (range >= config_.min_range && range <= config_.max_range); }

   private:
    VelodyneConfig config_;
    std::vector<FullCloudPtr> organized_raw_pointcloud_;

    float cos_vert_angle_table_[kVLP16ScanPerFiring]{};
    float sin_vert_angle_table_[kVLP16ScanPerFiring]{};
    const double (*inner_time_)[kBlocksPerPacket][kScanPerBlock];
    const float (*azimuth_corrected_table_)[kVLP16FiringsPerBlock][kVLP16ScanPerFiring];

    std::vector<int> rings_map_;
    float sin_rot_table_[kRotationMaxUnits]{};
    float cos_rot_table_[kRotationMaxUnits]{};
};

}  // namespace sad::tools
