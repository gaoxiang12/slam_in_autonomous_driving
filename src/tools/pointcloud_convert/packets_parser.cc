#include "tools/pointcloud_convert/packets_parser.h"
#include <glog/logging.h>

namespace sad::tools {

float transform_x_of_x, transform_y_of_x, transform_z_of_x;
float transform_x_of_y, transform_y_of_y, transform_z_of_y;
float transform_x_of_z, transform_y_of_z, transform_z_of_z;

int PacketsParser::Setup(const VelodyneConfig &conf) {
    config_ = conf;

    for (auto &rings_pointcloud : organized_raw_pointcloud_) {
        rings_pointcloud = boost::make_shared<FullPointCloudType>();
    }

    double tmp_min_angle = config_.view_direction + config_.view_width / 2;
    double tmp_max_angle = config_.view_direction - config_.view_width / 2;

    tmp_min_angle = fmod(fmod(tmp_min_angle, 2 * M_PI) + 2 * M_PI, 2 * M_PI);
    tmp_max_angle = fmod(fmod(tmp_max_angle, 2 * M_PI) + 2 * M_PI, 2 * M_PI);

    config_.min_angle = 100 * (2 * M_PI - tmp_min_angle) * 180 / M_PI + 0.5;
    config_.max_angle = 100 * (2 * M_PI - tmp_max_angle) * 180 / M_PI + 0.5;

    if (config_.min_angle == config_.max_angle) {
        config_.min_angle = 0;
        config_.max_angle = 36000;
    }

    for (uint16_t rot_index = 0; rot_index < kRotationMaxUnits; ++rot_index) {
        float rotation = angles::from_degrees(kRotationResolution * rot_index);
        cos_rot_table_[rot_index] = cosf(rotation);
        sin_rot_table_[rot_index] = sinf(rotation);
    }

    for (int i = 0; i < kVLP16ScanPerFiring; ++i) {
        cos_vert_angle_table_[i] = cosf(kVLP16ScanVertAngle[i]);
        sin_vert_angle_table_[i] = sinf(kVLP16ScanVertAngle[i]);
    }

    std::map<double, int> ordered_vert_angle;
    for (int i = 0; i < kVLP16ScanPerFiring; ++i) {
        ordered_vert_angle[kVLP16ScanVertAngle[i]] = i;
    }

    int index = 0;
    rings_map_.resize(ordered_vert_angle.size());
    for (auto iter = ordered_vert_angle.begin(); iter != ordered_vert_angle.end(); ++iter, index++) {
        rings_map_[iter->second] = index;
    }

    int xr = config_.roll >= 0 ? config_.roll * 100 : (config_.roll + 360) * 100;
    int yr = config_.pitch >= 0 ? config_.pitch * 100 : (config_.pitch + 360) * 100;
    int zr = config_.yaw >= 0 ? config_.yaw * 100 : (config_.yaw + 360) * 100;

    transform_x_of_x = cos_rot_table_[zr] * cos_rot_table_[yr];
    transform_y_of_x =
        sin_rot_table_[zr] * cos_rot_table_[xr] + cos_rot_table_[zr] * sin_rot_table_[yr] * sin_rot_table_[xr];
    transform_z_of_x =
        sin_rot_table_[zr] * sin_rot_table_[xr] - cos_rot_table_[zr] * sin_rot_table_[yr] * cos_rot_table_[xr];

    transform_x_of_y = -sin_rot_table_[zr] * cos_rot_table_[yr];
    transform_y_of_y =
        cos_rot_table_[zr] * cos_rot_table_[xr] - sin_rot_table_[zr] * sin_rot_table_[yr] * sin_rot_table_[xr];
    transform_z_of_y =
        cos_rot_table_[zr] * sin_rot_table_[xr] + sin_rot_table_[zr] * sin_rot_table_[yr] * cos_rot_table_[xr];

    transform_x_of_z = sin_rot_table_[yr];
    transform_y_of_z = -cos_rot_table_[yr] * sin_rot_table_[xr];
    transform_z_of_z = cos_rot_table_[yr] * cos_rot_table_[xr];

    return 0;
}

void PacketsParser::PaddingPointCloud(const PacketsMsgPtr &scan_msg, FullCloudPtr &out_pc_msg_ptr) {
    size_t packets_size = scan_msg->packets.size();

    if (packets_size <= 0) {
        LOG(ERROR) << "velodyne pointcloud node input packets size is empty";
        return;
    }

    out_pc_msg_ptr->reserve(packets_size * kScanPerBlock);

    int one_ring_points_size = packets_size * kScanPerBlock / organized_raw_pointcloud_.size();
    for (auto &vert_ring_pointcloud : organized_raw_pointcloud_) {
        vert_ring_pointcloud->points.clear();
        vert_ring_pointcloud->points.reserve(one_ring_points_size);
    }

    for (size_t i = 0; i < packets_size; ++i) {
        Unpack(scan_msg->packets[i], scan_msg->header.stamp, organized_raw_pointcloud_);
    }

    ArrangePointcloud(organized_raw_pointcloud_, out_pc_msg_ptr);

    out_pc_msg_ptr->header.stamp = pcl_conversions::toPCL(scan_msg->header).stamp;
    out_pc_msg_ptr->header.frame_id = scan_msg->header.frame_id;

    if (out_pc_msg_ptr->empty()) {
        //        LOG(ERROR) << "All points is NAN!Please check velodyne:" << config_.model;
    }
}

void PacketsParser::Unpack(const velodyne_msgs::VelodynePacket &pkt, const ros::Time &msg_stamp,
                           std::vector<FullCloudPtr> &rings_pointcloud) {
    float azimuth_diff = 0.0;
    float last_azimuth_diff = 0.0;
    float azimuth_corrected_f = 0.0;
    int azimuth_corrected = 0;

    const double pkt_time = ros::Time(pkt.stamp).toSec();
    const auto *raw = (const RawPacket *)&pkt.data[0];

    for (int block = 0; block < kBlocksPerPacket; block++) {
        if (kUpperBank != raw->blocks[block].header) {
            return;
        }

        auto azimuth = static_cast<float>(raw->blocks[block].rotation);

        if (block < (kBlocksPerPacket - 1)) {
            azimuth_diff = (float)((36000 + raw->blocks[block + 1].rotation - raw->blocks[block].rotation) % 36000);
            last_azimuth_diff = azimuth_diff;
        } else {
            azimuth_diff = last_azimuth_diff;
        }

        for (int firing = 0, k = 0; firing < kVLP16FiringsPerBlock; ++firing) {
            for (int dsr = 0; dsr < kVLP16ScanPerFiring; ++dsr, k += kRawScanSize) {
                union TwoBytes raw_distance;
                raw_distance.bytes[0] = raw->blocks[block].data[k];
                raw_distance.bytes[1] = raw->blocks[block].data[k + 1];

                azimuth_corrected_f = azimuth + (*azimuth_corrected_table_)[firing][dsr];
                azimuth_corrected = ((int)round(azimuth_corrected_f)) % 36000;

                FullPointType point;

                double point_stamp_us = pkt_time * 1e6 + (*inner_time_)[block][firing * kVLP16ScanPerFiring + dsr];

                point.time = static_cast<float>(point_stamp_us - msg_stamp.toSec() * 1e6);

                int ring = rings_map_[dsr];
                point.ring = ring;
                // point.angle = azimuth_corrected * 1.0 / 100;
                // point.angle = point.angle >= 0.0 ? point.angle : point.angle + 360.0;
                // point.angle = point.angle <= 360.0 ? point.angle : point.angle - 360.0;

                point.intensity = raw->blocks[block].data[k + 2];

                float distance = raw_distance.uint * kDistanceResolution;

                if (raw_distance.uint == 0 || !isScanValid(azimuth_corrected, distance)) {
                    FilledNAN(point);
                } else {
                    ComputeCoords(distance, dsr, azimuth_corrected, point);
                    if (!isScanValid(point)) {
                        FilledFree(point);
                    }
                }

                rings_pointcloud[ring]->points.emplace_back(point);
                ++rings_pointcloud[ring]->width;
            }
        }
    }
}

void PacketsParser::ArrangePointcloud(const std::vector<FullCloudPtr> &rings_pointcloud, FullCloudPtr &out_pc_ptr) {
    int width = 0;
    int height = 0;
    for (size_t i = 0; i < rings_pointcloud.size(); ++i) {
        if (rings_pointcloud[i]->points.size() <= 0) {
            ROS_ERROR_STREAM("lidar points in ring " << i << " is empty");
        }

        if (config_.is_organized) {
            out_pc_ptr->insert(out_pc_ptr->end(), rings_pointcloud[i]->points.begin(),
                               rings_pointcloud[i]->points.end());
            width = std::max(static_cast<int>(rings_pointcloud[i]->points.size()), width);
            height += 1;
        } else {
            for (auto &point : *rings_pointcloud[i]) {
                if (isPointValid(point)) {
                    out_pc_ptr->points.emplace_back(point);
                    width += 1;
                }
            }

            height = 1;
        }
    }

    out_pc_ptr->width = width;
    out_pc_ptr->height = height;
}

inline void PacketsParser::FilledNAN(FullPointType &point) {
    point.x = HIT_NAN;
    point.y = HIT_NAN;
    point.z = HIT_NAN;
    point.intensity = 0;
}

inline void PacketsParser::FilledFree(FullPointType &point) {
    point.x = HIT_FREE;
    point.y = HIT_FREE;
    point.z = HIT_FREE;
    point.intensity = 0;
}

inline bool PacketsParser::isPointValid(const FullPointType &point) {
    if (point.x == HIT_NAN || point.x == HIT_FREE || point.y == HIT_NAN || point.y == HIT_FREE || point.z == HIT_NAN ||
        point.z == HIT_FREE)
        return false;

    return true;
}

inline bool PacketsParser::isScanValid(int rotation, float range) {
    if (range < config_.min_range || range > config_.max_range) {
        return false;
    }

    return true;
}

inline bool PacketsParser::isScanValid(const FullPointType &point) {
    if (point.x < config_.car_front && point.x > config_.car_back && point.y < config_.car_left &&
        point.y > config_.car_right && point.z < config_.car_top && point.z > config_.car_bottom) {
        return false;
    }

    return true;
}

void PacketsParser::ComputeCoords(const float distance, const int vert_line_index, const uint16_t &rotation,
                                  FullPointType &point) {
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;

    float cos_rot_angle = cos_rot_table_[rotation];
    float sin_rot_angle = sin_rot_table_[rotation];

    float xy_distance = distance * cos_vert_angle_table_[vert_line_index];

    x = xy_distance * sin_rot_angle;
    y = xy_distance * cos_rot_angle;
    z = distance * sin_vert_angle_table_[vert_line_index];

    point.x = y;
    point.y = -x;
    point.z = z;

    if (config_.enable_coordinate_transformation) {
        float x_coord = point.x;
        float y_coord = point.y;
        float z_coord = point.z;
        point.x = transform_x_of_x * x_coord + transform_y_of_x * y_coord + transform_z_of_x * z_coord;
        point.y = transform_x_of_y * x_coord + transform_y_of_y * y_coord + transform_z_of_y * z_coord;
        point.z = transform_x_of_z * x_coord + transform_y_of_z * y_coord + transform_z_of_z * z_coord;

        point.x = point.x + config_.xoffset;
        point.y = point.y + config_.yoffset;
        point.z = point.z + config_.zoffset;
    }
}

}  // namespace sad::tools