//
// Created by xiang on 2021/11/19.
//

#ifndef FUSION_VELODYNE_CONFIG_H
#define FUSION_VELODYNE_CONFIG_H

namespace sad::tools {

/// velodyne的配置参数
struct VelodyneConfig {
    int type = 16;
    double xoffset = 0.333;
    double yoffset = 0;
    double zoffset = 1.17;
    double roll = -0.085;
    double pitch = 0;
    double yaw = 90.7557;

    double max_range = 60.0;
    double min_range = 1.0;
    double max_angle = 0;
    double min_angle = 0;
    bool is_organized = false;
    double view_direction = 0;
    double view_width = 0;
    bool enable_coordinate_transformation = true;
    double car_left = 1.0;
    double car_right = -1.0;
    double car_front = 2.0;
    double car_back = -2.0;
    double car_top = 2.0;
    double car_bottom = -20.0;
};
}  // namespace sad::tools

#endif  // FUSION_VELODYNE_CONFIG_H
