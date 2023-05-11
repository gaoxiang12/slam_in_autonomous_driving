//
// Created by BowenBZ on 2023/5/10.
//

#include "common/point_cloud_utils.h"
#include "common/point_types.h"

#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>

/// 点云的一些工具函数

namespace sad {

/// 体素滤波
void VoxelGrid(CloudPtr cloud, float voxel_size) {
    pcl::VoxelGrid<sad::PointType> voxel;
    voxel.setLeafSize(voxel_size, voxel_size, voxel_size);
    voxel.setInputCloud(cloud);

    CloudPtr output(new PointCloudType);
    voxel.filter(*output);
    cloud->swap(*output);
}

/// 移除地面
void RemoveGround(CloudPtr cloud, float z_min) {
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

/// 写点云文件
template<typename CloudType> 
void SaveCloudToFile(const std::string &filePath, CloudType &cloud) {
    cloud.height = 1;
    cloud.width = cloud.size();
    pcl::io::savePCDFileASCII(filePath, cloud);
}

template void SaveCloudToFile<PointCloudType>(const std::string &filePath, PointCloudType &cloud);

template void SaveCloudToFile<FullPointCloudType>(const std::string &filePath, FullPointCloudType &cloud);

}  // namespace sad