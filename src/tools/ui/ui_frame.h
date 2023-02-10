//
// Created by xiang on 22-8-25.
//

#ifndef SLAM_IN_AUTO_DRIVING_UI_FRAME_H
#define SLAM_IN_AUTO_DRIVING_UI_FRAME_H

#include "common/eigen_types.h"
#include "common/point_types.h"

#include <pcl/filters/voxel_grid.h>

namespace sad {

struct UiFrame {
    SE3 Twb_;
    CloudPtr scan_body_ = nullptr;      // baselink frame下的点云
    UiCloudPtr cloud_world_ = nullptr;  // odom frame下经过降采样的点云

    UiFrame() = default;

    UiFrame(const CloudPtr &cloud, const SE3 &T) {
        scan_body_ = cloud;
        Twb_ = T;
    }

    void Convert(const std::shared_ptr<pcl::VoxelGrid<PointType>> &voxel_filter) {
        auto tmp = boost::make_shared<PointCloudType>();
        voxel_filter->setInputCloud(scan_body_);
        voxel_filter->filter(*tmp);

        cloud_world_ = boost::make_shared<UiPointCloudType>();
        cloud_world_->reserve(tmp->size());

        // 降采样后的scan，转换坐标系，着色
        auto rot = Twb_.rotationMatrix().cast<float>().eval();
        auto trans = Twb_.translation().cast<float>().eval();

        for (const auto &p : tmp->points) {
            UiPointType q;
            q.getVector3fMap() = rot * p.getVector3fMap() + trans;
            q.r = uint8_t(128 + p.intensity * 6);
            cloud_world_->points.emplace_back(q);
        }

        cloud_world_->header = tmp->header;
    }
};
}  // namespace sad

#endif  // SLAM_IN_AUTO_DRIVING_UI_FRAME_H
