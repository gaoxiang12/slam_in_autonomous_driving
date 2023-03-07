//
// Created by xiang on 22-9-7.
//

#include "ui_cloud.h"

#include <execution>
#include <numeric>

namespace sad::ui {

std::vector<Vec4f> UiCloud::intensity_color_table_pcl_;

UiCloud::UiCloud(CloudPtr cloud) { SetCloud(cloud, SE3()); }

void UiCloud::SetCloud(CloudPtr cloud, const SE3& pose) {
    if (intensity_color_table_pcl_.empty()) {
        BuildIntensityTable();
    }

    assert(cloud != nullptr && cloud->empty() == false);
    xyz_data_.resize(cloud->size());
    color_data_pcl_.resize(cloud->size());
    color_data_intensity_.resize(cloud->size());
    color_data_height_.resize(cloud->size());
    color_data_gray_.resize(cloud->size());

    std::vector<int> idx(cloud->size());
    std::iota(idx.begin(), idx.end(), 0);

    SE3f pose_f = pose.cast<float>();
    std::for_each(std::execution::par_unseq, idx.begin(), idx.end(), [&](const int& id) {
        const auto& pt = cloud->points[id];
        auto pt_world = pose_f * cloud->points[id].getVector3fMap();
        xyz_data_[id] = Vec3f(pt_world.x(), pt_world.y(), pt_world.z());
        color_data_pcl_[id] = IntensityToRgbPCL(pt.intensity);
        color_data_gray_[id] = Vec4f(0.5, 0.5, 0.5, 0.2);
        color_data_height_[id] = IntensityToRgbPCL(pt.z * 10);
        color_data_intensity_[id] =
            Vec4f(pt.intensity / 255.0 * 3.0, pt.intensity / 255.0 * 3.0, pt.intensity / 255.0 * 3.0, 0.2);
    });

    vbo_ = pangolin::GlBuffer(pangolin::GlArrayBuffer, xyz_data_);
}

void UiCloud::Render() {
    if (vbo_.IsValid() && cbo_.IsValid()) {
        pangolin::RenderVboCbo(vbo_, cbo_);
    }
}

void UiCloud::BuildIntensityTable() {
    intensity_color_table_pcl_.reserve(255 * 6);
    auto make_color = [](int r, int g, int b) -> Vec4f { return Vec4f(r / 255.0f, g / 255.0f, b / 255.0f, 0.2f); };
    for (int i = 0; i < 256; i++) {
        intensity_color_table_pcl_.emplace_back(make_color(255, i, 0));
    }
    for (int i = 0; i < 256; i++) {
        intensity_color_table_pcl_.emplace_back(make_color(255 - i, 0, 255));
    }
    for (int i = 0; i < 256; i++) {
        intensity_color_table_pcl_.emplace_back(make_color(0, 255, i));
    }
    for (int i = 0; i < 256; i++) {
        intensity_color_table_pcl_.emplace_back(make_color(255, 255 - i, 0));
    }
    for (int i = 0; i < 256; i++) {
        intensity_color_table_pcl_.emplace_back(make_color(i, 0, 255));
    }
    for (int i = 0; i < 256; i++) {
        intensity_color_table_pcl_.emplace_back(make_color(0, 255, 255 - i));
    }
}

void UiCloud::SetRenderColor(UiCloud::UseColor use_color) {
    use_color_ = use_color;

    if (use_color_ == UseColor::PCL_COLOR) {
        cbo_ = pangolin::GlBuffer(pangolin::GlArrayBuffer, color_data_pcl_);
    } else if (use_color_ == UseColor::INTENSITY_COLOR) {
        cbo_ = pangolin::GlBuffer(pangolin::GlArrayBuffer, color_data_intensity_);
    } else if (use_color_ == UseColor::HEIGHT_COLOR) {
        cbo_ = pangolin::GlBuffer(pangolin::GlArrayBuffer, color_data_height_);
    } else if (use_color_ == UseColor::GRAY_COLOR) {
        cbo_ = pangolin::GlBuffer(pangolin::GlArrayBuffer, color_data_gray_);
    }
}

}  // namespace sad::ui