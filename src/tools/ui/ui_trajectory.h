//
// Created by xiang on 22-9-7.
//

#ifndef SAD_UI_TRAJECTORY_H
#define SAD_UI_TRAJECTORY_H

#include "common/eigen_types.h"

#include <pangolin/gl/glvbo.h>

namespace sad::ui {

/// UI中的轨迹绘制
class UiTrajectory {
   public:
    UiTrajectory(const Vec3f& color) : color_(color) { pos_.reserve(max_size_); }

    /// 增加一个轨迹点
    void AddPt(const SE3& pose);

    /// 渲染此轨迹
    void Render();

    void Clear() {
        pos_.clear();
        pos_.reserve(max_size_);
        vbo_.Free();
    }

   private:
    int max_size_ = 1e6;           // 记录的最大点数
    std::vector<Vec3f> pos_;       // 轨迹记录数据
    Vec3f color_ = Vec3f::Zero();  // 轨迹颜色显示
    pangolin::GlBuffer vbo_;       // 显存顶点信息
};

}  // namespace sad::ui

#endif  // TFUSION_UI_TRAJECTORY_H
