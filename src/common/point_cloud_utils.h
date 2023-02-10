//
// Created by xiang on 2021/8/25.
//

#ifndef SLAM_IN_AUTO_DRIVING_POINT_CLOUD_UTILS_H
#define SLAM_IN_AUTO_DRIVING_POINT_CLOUD_UTILS_H

#include "common/point_types.h"

#include <pcl/filters/voxel_grid.h>

/// 点云的一些工具函数

namespace sad {

/// 体素滤波
void VoxelGrid(CloudPtr cloud, float voxel_size = 0.05) {
    pcl::VoxelGrid<sad::PointType> voxel;
    voxel.setLeafSize(voxel_size, voxel_size, voxel_size);
    voxel.setInputCloud(cloud);

    CloudPtr output(new PointCloudType);
    voxel.filter(*output);
    cloud->swap(*output);
}

/// 移除地面
void RemoveGround(CloudPtr cloud, float z_min = 0.5) {
    CloudPtr output(new PointCloudType);
    for (const auto& pt : cloud->points) {
        if (pt.z > z_min) {
            output->points.emplace_back(pt);
        }
    }

    output->height = 1;
    output->is_dense = false;
    output->width = output->points.size();
    cloud->swap(*output);
}

}  // namespace sad

#endif  // SLAM_IN_AUTO_DRIVING_POINT_CLOUD_UTILS_H
