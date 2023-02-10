#ifndef DRIVER_VELODYNE_POINT_TYPES_H
#define DRIVER_VELODYNE_POINT_TYPES_H

#include <pcl/point_types.h>

#include <limits>

namespace driver {
namespace velodyne {

const float HIT_NAN = std::numeric_limits<float>::max();
const float HIT_FREE = std::numeric_limits<float>::min();

/** Euclidean Velodyne coordinate, including intensity and ring number.
 * vertiacal angle, timestamp*/
struct PointXYZRRIAR {
    PCL_ADD_POINT4D;    // quad-word XYZ
    float range;        // range = (x^2+y^2+z^2)^0.5
    float radius;       // radisu = (x^2+y^2)^0.5
    uint8_t intensity;  // laser intensity reading
    uint16_t angle;     // azimuth angle, 0.01 degree
    uint8_t ring;       // laser ring number
    double timestamp;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW  // ensure proper alignment
} EIGEN_ALIGN16;

struct PointXYZRRIARL {
    PCL_ADD_POINT4D;                 // quad-word XYZ
    float range;                     // range = (x^2+y^2+z^2)^0.5
    float radius;                    // radisu = (x^2+y^2)^0.5
    uint8_t intensity;               // laser intensity reading
    uint16_t angle;                  // azimuth angle, 0.01 degree
    uint8_t ring;                    // laser ring number
    uint8_t label;                   // point label
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW  // ensure proper alignment
} EIGEN_ALIGN16;

}  // namespace velodyne
}  // namespace driver

POINT_CLOUD_REGISTER_POINT_STRUCT(
    driver::velodyne::PointXYZRRIAR,
    (float, x, x)(float, y, y)(float, z, z)(float, range, range)(float, radius, radius)(
        std::uint8_t, intensity, intensity)(std::uint16_t, angle, angle)(std::uint8_t, ring, ring))

POINT_CLOUD_REGISTER_POINT_STRUCT(driver::velodyne::PointXYZRRIARL,
                                  (float, x, x)(float, y, y)(float, z, z)(float, range, range)(
                                      float, radius, radius)(std::uint8_t, intensity, intensity)(
                                      std::uint16_t, angle,
                                      angle)(std::uint8_t, ring, ring)(std::uint8_t, label, label))

#endif  // __VELODYNE_POINTCLOUD_POINT_TYPES_H
