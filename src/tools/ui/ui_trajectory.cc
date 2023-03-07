//
// Created by xiang on 22-9-7.
//

#include "ui_trajectory.h"

namespace sad::ui {

void UiTrajectory::AddPt(const SE3& pose) {
    pos_.emplace_back(pose.translation().cast<float>());
    if (pos_.size() > max_size_) {
        pos_.erase(pos_.begin(), pos_.begin() + pos_.size() / 2);  // 删掉前一半的点
    }
    vbo_ = pangolin::GlBuffer(pangolin::GlArrayBuffer, pos_);
}

void UiTrajectory::Render() {
    if (!vbo_.IsValid()) {
        return;
    }

    glColor3f(color_[0], color_[1], color_[2]);

    // 点线形式
    glLineWidth(3.0);
    pangolin::RenderVbo(vbo_, GL_LINE_STRIP);
    glLineWidth(1.0);

    glPointSize(5.0);
    pangolin::RenderVbo(vbo_, GL_POINTS);
    glPointSize(1.0);
}

}  // namespace sad::ui