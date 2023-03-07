//
// Created by xiang on 22-9-7.
//

#ifndef SAD_UI_CAR_H
#define SAD_UI_CAR_H

#include <pangolin/gl/glvbo.h>

#include "common/eigen_types.h"

namespace sad::ui {

/// 在UI里显示的小车
class UiCar {
   public:
    UiCar(const Vec3f& color) : color_(color) {}

    /// 设置小车 Pose，重设显存中的点
    void SetPose(const SE3& pose);

    /// 渲染小车
    void Render();

   private:
    Vec3f color_;
    pangolin::GlBuffer vbo_;  // buffer data

    static std::vector<Vec3f> car_vertices_;  // 小车的顶点
};

}  // namespace sad::ui

#endif
