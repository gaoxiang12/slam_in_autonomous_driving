#include <gtest/gtest.h>

#include <iostream>

#include <Eigen/Dense>

#include "velodyne_lidar_model.h"

using namespace driver::velodyne;

class VelodyneLidarModelTest : public testing::Test {
 public:
  virtual void SetUp() {
    printf("VelodyneLidarModelTest setup");
    vlp_16 = new driver::velodyne::VelodyneVLP16();
    vlp_32 = new driver::velodyne::VelodyneVLP32();
    vlp_hdl32 = new driver::velodyne::VelodyneVLPHDL32();
  }
  virtual void TearDown() { printf("TearDown"); }

  driver::velodyne::VelodyneBase* vlp_16;
  driver::velodyne::VelodyneBase* vlp_32;
  driver::velodyne::VelodyneVLPHDL32* vlp_hdl32;
};

TEST_F(VelodyneLidarModelTest, TransformTestHDL32) {
  const std::array<float, kLaserPerFiring>& azimuth_corrected_table =
      vlp_hdl32->get_azimuth_corrected_table();
  const std::array<int, kLaserPerFiring>& azimuth_offsets =
      vlp_hdl32->get_azimuth_offset_table();
  const std::array<int, kLaserPerFiring>& firing_sequence =
      vlp_hdl32->get_firing_laser_sequence();
  const float& distance_resolution = vlp_hdl32->get_distance_resolution();
  const std::array<float, kLaserPerFiring>& vert_angle_table_cos =
      vlp_hdl32->get_vert_angle_table_cos();
  const std::array<float, kLaserPerFiring>& vert_angle_table_sin =
      vlp_hdl32->get_vert_angle_table_sin();
  const std::array<float, kRotationMaxUnits>& rot_table_cos =
      vlp_hdl32->get_rot_table_cos();
  const std::array<float, kRotationMaxUnits>& rot_table_sin =
      vlp_hdl32->get_rot_table_sin();

  std::cout << "azimuth corrected table:\n";
  std::cout << "distance resolution:" << distance_resolution << std::endl;
  for (size_t laser = 0; laser < kLaserPerFiring; ++laser) {
    std::cout << laser << ":" << firing_sequence[laser] << ", "
              << azimuth_offsets[laser] << ", "
              << azimuth_corrected_table[laser] << std::endl;
  }
}

TEST_F(VelodyneLidarModelTest, TransformTest32) {
  const std::array<float, kLaserPerFiring>& azimuth_corrected_table =
      vlp_32->get_azimuth_corrected_table();
  const std::array<int, kLaserPerFiring>& azimuth_offsets =
      vlp_32->get_azimuth_offset_table();
  const std::array<int, kLaserPerFiring>& firing_sequence =
      vlp_32->get_firing_laser_sequence();
  const float& distance_resolution = vlp_32->get_distance_resolution();
  const std::array<float, kLaserPerFiring>& vert_angle_table_cos =
      vlp_32->get_vert_angle_table_cos();
  const std::array<float, kLaserPerFiring>& vert_angle_table_sin =
      vlp_32->get_vert_angle_table_sin();
  const std::array<float, kRotationMaxUnits>& rot_table_cos =
      vlp_32->get_rot_table_cos();
  const std::array<float, kRotationMaxUnits>& rot_table_sin =
      vlp_32->get_rot_table_sin();

  std::cout << "azimuth corrected table:\n";
  std::cout << "distance resolution:" << distance_resolution << std::endl;
  for (size_t laser = 0; laser < kLaserPerFiring; ++laser) {
    std::cout << laser << ":" << firing_sequence[laser] << ", "
              << azimuth_offsets[laser] << ", "
              << azimuth_corrected_table[laser] << std::endl;
  }
}

TEST_F(VelodyneLidarModelTest, TransformTest16) {
  const std::array<float, kLaserPerFiring>& azimuth_corrected_table =
      vlp_16->get_azimuth_corrected_table();
  const std::array<int, kLaserPerFiring>& azimuth_offsets =
      vlp_16->get_azimuth_offset_table();
  const std::array<int, kLaserPerFiring>& firing_sequence =
      vlp_16->get_firing_laser_sequence();
  const float& distance_resolution = vlp_16->get_distance_resolution();
  const std::array<float, kLaserPerFiring>& vert_angle_table_cos =
      vlp_16->get_vert_angle_table_cos();
  const std::array<float, kLaserPerFiring>& vert_angle_table_sin =
      vlp_16->get_vert_angle_table_sin();
  const std::array<float, kRotationMaxUnits>& rot_table_cos =
      vlp_16->get_rot_table_cos();
  const std::array<float, kRotationMaxUnits>& rot_table_sin =
      vlp_16->get_rot_table_sin();

  std::cout << "azimuth corrected table:\n";
  std::cout << "distance resolution:" << distance_resolution << std::endl;
  for (size_t laser = 0; laser < kLaserPerFiring; ++laser) {
    std::cout << laser << ":" << firing_sequence[laser] << ", "
              << azimuth_offsets[laser] << ", "
              << azimuth_corrected_table[laser] << std::endl;
  }
}

// The main entrance of gtest:
int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}