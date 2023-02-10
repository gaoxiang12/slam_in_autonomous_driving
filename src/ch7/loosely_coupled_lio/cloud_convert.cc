#include "cloud_convert.h"

#include <glog/logging.h>
#include <yaml-cpp/yaml.h>
#include <execution>

namespace sad {

void CloudConvert::Process(const livox_ros_driver::CustomMsg::ConstPtr &msg, FullCloudPtr &pcl_out) {
    AviaHandler(msg);
    *pcl_out = cloud_out_;
}

void CloudConvert::Process(const sensor_msgs::PointCloud2::ConstPtr &msg, FullCloudPtr &pcl_out) {
    switch (lidar_type_) {
        case LidarType::OUST64:
            Oust64Handler(msg);
            break;

        case LidarType::VELO32:
            VelodyneHandler(msg);
            break;

        default:
            LOG(ERROR) << "Error LiDAR Type: " << int(lidar_type_);
            break;
    }
    *pcl_out = cloud_out_;
}

void CloudConvert::AviaHandler(const livox_ros_driver::CustomMsg::ConstPtr &msg) {
    cloud_out_.clear();
    cloud_full_.clear();
    int plsize = msg->point_num;

    cloud_out_.reserve(plsize);
    cloud_full_.resize(plsize);

    std::vector<bool> is_valid_pt(plsize, false);
    std::vector<uint> index(plsize - 1);
    for (uint i = 0; i < plsize - 1; ++i) {
        index[i] = i + 1;  // 从1开始
    }

    std::for_each(std::execution::par_unseq, index.begin(), index.end(), [&](const uint &i) {
        if ((msg->points[i].line < num_scans_) &&
            ((msg->points[i].tag & 0x30) == 0x10 || (msg->points[i].tag & 0x30) == 0x00)) {
            if (i % point_filter_num_ == 0) {
                cloud_full_[i].x = msg->points[i].x;
                cloud_full_[i].y = msg->points[i].y;
                cloud_full_[i].z = msg->points[i].z;
                cloud_full_[i].intensity = msg->points[i].reflectivity;
                cloud_full_[i].time = msg->points[i].offset_time / float(1000000);

                if ((abs(cloud_full_[i].x - cloud_full_[i - 1].x) > 1e-7) ||
                    (abs(cloud_full_[i].y - cloud_full_[i - 1].y) > 1e-7) ||
                    (abs(cloud_full_[i].z - cloud_full_[i - 1].z) > 1e-7)) {
                    is_valid_pt[i] = true;
                }
            }
        }
    });

    for (uint i = 1; i < plsize; i++) {
        if (is_valid_pt[i]) {
            cloud_out_.points.push_back(cloud_full_[i]);
        }
    }
}

void CloudConvert::Oust64Handler(const sensor_msgs::PointCloud2::ConstPtr &msg) {
    cloud_out_.clear();
    cloud_full_.clear();
    pcl::PointCloud<ouster_ros::Point> pl_orig;
    pcl::fromROSMsg(*msg, pl_orig);
    int plsize = pl_orig.size();
    cloud_out_.reserve(plsize);

    for (int i = 0; i < pl_orig.points.size(); i++) {
        if (i % point_filter_num_ != 0) continue;

        double range = pl_orig.points[i].x * pl_orig.points[i].x + pl_orig.points[i].y * pl_orig.points[i].y +
                       pl_orig.points[i].z * pl_orig.points[i].z;

        Vec3d pt_vec;
        FullPointType added_pt;
        added_pt.x = pl_orig.points[i].x;
        added_pt.y = pl_orig.points[i].y;
        added_pt.z = pl_orig.points[i].z;
        added_pt.intensity = pl_orig.points[i].intensity;
        added_pt.time = pl_orig.points[i].t / 1e6;  // curvature unit: ms

        cloud_out_.points.push_back(added_pt);
    }
}

void CloudConvert::VelodyneHandler(const sensor_msgs::PointCloud2::ConstPtr &msg) {
    cloud_out_.clear();
    cloud_full_.clear();

    pcl::PointCloud<velodyne_ros::Point> pl_orig;
    pcl::fromROSMsg(*msg, pl_orig);
    int plsize = pl_orig.points.size();
    cloud_out_.reserve(plsize);

    double omega_l = 3.61;  // scan angular velocity
    std::vector<bool> is_first(num_scans_, true);
    std::vector<double> yaw_fp(num_scans_, 0.0);    // yaw of first scan point
    std::vector<float> yaw_last(num_scans_, 0.0);   // yaw of last scan point
    std::vector<float> time_last(num_scans_, 0.0);  // last offset time

    bool given_offset_time = false;
    if (pl_orig.points[plsize - 1].time > 0) {
        given_offset_time = true;
    } else {
        given_offset_time = false;

        double yaw_first = atan2(pl_orig.points[0].y, pl_orig.points[0].x) * 57.29578;
        double yaw_end = yaw_first;
        int layer_first = pl_orig.points[0].ring;
        for (uint i = plsize - 1; i > 0; i--) {
            if (pl_orig.points[i].ring == layer_first) {
                yaw_end = atan2(pl_orig.points[i].y, pl_orig.points[i].x) * 57.29578;
                break;
            }
        }
    }

    for (int i = 0; i < plsize; i++) {
        FullPointType added_pt;
        added_pt.x = pl_orig.points[i].x;
        added_pt.y = pl_orig.points[i].y;
        added_pt.z = pl_orig.points[i].z;
        added_pt.intensity = pl_orig.points[i].intensity;
        added_pt.time = pl_orig.points[i].time * time_scale_;  // curvature unit: ms

        /// 略掉过近的点
        if (added_pt.getVector3fMap().norm() < 4.0) {
            continue;
        }

        if (!given_offset_time) {
            int layer = pl_orig.points[i].ring;
            double yaw_angle = atan2(added_pt.y, added_pt.x) * 57.2957;

            if (is_first[layer]) {
                yaw_fp[layer] = yaw_angle;
                is_first[layer] = false;
                added_pt.time = 0.0;
                yaw_last[layer] = yaw_angle;
                time_last[layer] = added_pt.time;
                continue;
            }

            // compute offset time
            if (yaw_angle <= yaw_fp[layer]) {
                added_pt.time = (yaw_fp[layer] - yaw_angle) / omega_l;
            } else {
                added_pt.time = (yaw_fp[layer] - yaw_angle + 360.0) / omega_l;
            }

            if (added_pt.time < time_last[layer]) {
                added_pt.time += 360.0 / omega_l;
            }

            yaw_last[layer] = yaw_angle;
            time_last[layer] = added_pt.time;
        }

        if (i % point_filter_num_ == 0) {
            cloud_out_.points.push_back(added_pt);
        }
    }
}

void CloudConvert::LoadFromYAML(const std::string &yaml_file) {
    auto yaml = YAML::LoadFile(yaml_file);
    time_scale_ = yaml["preprocess"]["time_scale"].as<double>();
    int lidar_type = yaml["preprocess"]["lidar_type"].as<int>();
    num_scans_ = yaml["preprocess"]["scan_line"].as<int>();
    point_filter_num_ = yaml["point_filter_num"].as<int>();

    if (lidar_type == 1) {
        lidar_type_ = LidarType::AVIA;
        LOG(INFO) << "Using AVIA Lidar";
    } else if (lidar_type == 2) {
        lidar_type_ = LidarType::VELO32;
        LOG(INFO) << "Using Velodyne 32 Lidar";
    } else if (lidar_type == 3) {
        lidar_type_ = LidarType::OUST64;
        LOG(INFO) << "Using OUST 64 Lidar";
    } else {
        LOG(WARNING) << "unknown lidar_type";
    }
}

}  // namespace sad
