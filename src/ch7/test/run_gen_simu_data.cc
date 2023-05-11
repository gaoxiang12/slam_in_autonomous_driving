//
// Created by xiang on 2022/7/13.
//

#include <glog/logging.h>
#include "ch7/gen_simu_data.h"

#include <pcl/io/pcd_io.h>

#include "common/point_cloud_utils.h"

int main(int argc, char** argv) {
    sad::GenSimuData gen;
    gen.Gen();

    sad::SaveCloudToFile("./data/ch7/sim_source.pcd", *gen.GetSource());
    sad::SaveCloudToFile("./data/ch7/sim_target.pcd", *gen.GetTarget());

    SE3 T_target_source = gen.GetPose().inverse();
    LOG(INFO) << "gt pose: " << T_target_source.translation().transpose() << ", "
              << T_target_source.so3().unit_quaternion().coeffs().transpose();

    return 0;
}