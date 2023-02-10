#ifndef DRIVER_VELODYNE_LIDAR_TRANSFORM_H
#define DRIVER_VELODYNE_LIDAR_TRANSFORM_H

#include "point_types.h"
#include "velodyne_constant.h"
#include "velodyne_lidar_model.h"

namespace driver {
namespace velodyne {
class LidarTransform {
 public:
  LidarTransform(float roll, float pitch, float yaw, float x_offset,
                 float y_offset, float z_offset)
      : yaw_(yaw),
        x_offset_(x_offset),
        y_offset_(y_offset),
        z_offset_(z_offset),
        car_left_(0.0),
        car_right_(0.0),
        car_front_(0.0),
        car_back_(0.0) {
    const float roll_angle = roll / 180.0 * M_PI;
    const float pitch_angle = pitch / 180.0 * M_PI;
    const float yaw_angle = yaw / 180.0 * M_PI;
    const float roll_cos = cosf(roll_angle);
    const float roll_sin = sinf(roll_angle);
    const float pitch_cos = cosf(pitch_angle);
    const float pitch_sin = sinf(pitch_angle);
    const float yaw_cos = cosf(yaw_angle);
    const float yaw_sin = sinf(yaw_angle);
    //  transform x
    transform_x_of_x_ = yaw_cos * pitch_cos;
    transform_y_of_x_ = yaw_sin * roll_cos + yaw_cos * pitch_sin * roll_sin;
    transform_z_of_x_ = yaw_sin * roll_sin - yaw_cos * pitch_sin * roll_cos;
    //  transform y
    transform_x_of_y_ = -yaw_sin * pitch_cos;
    transform_y_of_y_ = yaw_cos * roll_cos - yaw_sin * pitch_sin * roll_sin;
    transform_z_of_y_ = yaw_cos * roll_sin + yaw_sin * pitch_sin * roll_cos;
    //  transform z
    transform_x_of_z_ = pitch_sin;
    transform_y_of_z_ = -pitch_cos * roll_sin;
    transform_z_of_z_ = pitch_cos * roll_cos;
  }

  void setRange(const float left, const float right, const float front,
                const float back) {
    car_left_ = left;
    car_right_ = right;
    car_front_ = front;
    car_back_ = back;
  }

  template <typename PoinT>
  bool IsInCarFootPrint(PoinT* point) {
    if (point->x <= car_front_ && point->x >= car_back_ &&
        point->y <= car_left_ && point->y >= car_right_) {
      point->x = point->y = point->z = point->range = point->radius = 0.0;
      return false;
    }
    return true;
  }

  template <typename PoinT>
  void TransformCloud(PoinT* point) {
    int angle = static_cast<int>(point->angle) + yaw_ / kRotationResolution;
    angle = (angle + kRotationMaxUnits) % kRotationMaxUnits;
    point->angle = static_cast<uint16_t>(angle);

    // hit free point, no need to calculate x,y,z,range,radius
    if (point->range >= kLaserHitFree) {
      return;
    }
    float x_coord = point->x;
    float y_coord = point->y;
    float z_coord = point->z;
    point->x = transform_x_of_x_ * x_coord + transform_y_of_x_ * y_coord +
               transform_z_of_x_ * z_coord;
    point->y = transform_x_of_y_ * x_coord + transform_y_of_y_ * y_coord +
               transform_z_of_y_ * z_coord;
    point->z = transform_x_of_z_ * x_coord + transform_y_of_z_ * y_coord +
               transform_z_of_z_ * z_coord;

    // translate point from lidar frame to back-wheel frame
    point->x = point->x + x_offset_;
    point->y = point->y + y_offset_;
    point->z = point->z + z_offset_;
    point->radius = std::hypot(point->x, point->y);
    point->range = std::hypot(point->radius, point->z);
  }

 private:
  float yaw_;
  float x_offset_;
  float y_offset_;
  float z_offset_;
  float car_left_;
  float car_right_;
  float car_front_;
  float car_back_;

  float transform_x_of_x_;
  float transform_y_of_x_;
  float transform_z_of_x_;
  float transform_x_of_y_;
  float transform_y_of_y_;
  float transform_z_of_y_;
  float transform_x_of_z_;
  float transform_y_of_z_;
  float transform_z_of_z_;
};
}  // namespace velodyne
}  // namespace driver
#endif  // DRIVER_VELODYNE_LIDAR_TRANSFORM_H