//
// Created by xiang on 22-12-7.
//

#include "keyframe.h"
#include <glog/logging.h>
#include <pcl/io/pcd_io.h>
#include <iomanip>
#include "common/point_cloud_utils.h"

namespace sad {

void Keyframe::SaveAndUnloadScan(const std::string &path) {
    if (cloud_) {
        sad::SaveCloudToFile(path + "/" + std::to_string(id_) + ".pcd", *cloud_);
        cloud_ = nullptr;
    }
}

void Keyframe::LoadScan(const std::string &path) {
    cloud_.reset(new PointCloudType);
    pcl::io::loadPCDFile(path + "/" + std::to_string(id_) + ".pcd", *cloud_);
}

void Keyframe::Save(std::ostream &os) {
    auto save_SE3 = [](std::ostream &f, SE3 pose) {
        auto q = pose.so3().unit_quaternion();
        Vec3d t = pose.translation();
        f << t[0] << " " << t[1] << " " << t[2] << " " << q.x() << " " << q.y() << " " << q.z() << " " << q.w() << " ";
    };
    os << id_ << " " << std::setprecision(18) << timestamp_ << " " << rtk_heading_valid_ << " " << rtk_valid_ << " "
       << rtk_inlier_ << " ";
    save_SE3(os, lidar_pose_);
    save_SE3(os, rtk_pose_);
    save_SE3(os, opti_pose_1_);
    save_SE3(os, opti_pose_2_);
    os << std::endl;
}

void Keyframe::Load(std::istream &is) {
    is >> id_ >> timestamp_ >> rtk_heading_valid_ >> rtk_valid_ >> rtk_inlier_;

    auto load_SE3 = [](std::istream &f) -> SE3 {
        SE3 ret;
        double q[4];
        double t[3];
        f >> t[0] >> t[1] >> t[2] >> q[0] >> q[1] >> q[2] >> q[3];
        return SE3(Quatd(q[3], q[0], q[1], q[2]), Vec3d(t[0], t[1], t[2]));
    };
    lidar_pose_ = load_SE3(is);
    rtk_pose_ = load_SE3(is);
    opti_pose_1_ = load_SE3(is);
    opti_pose_2_ = load_SE3(is);
}

bool LoadKeyFrames(const std::string &path, std::map<IdType, std::shared_ptr<Keyframe>> &keyframes) {
    std::ifstream fin(path);
    if (!fin) {
        return false;
    }

    while (!fin.eof()) {
        std::string line;
        std::getline(fin, line);

        if (line.empty()) {
            break;
        }

        std::stringstream ss;
        ss << line;
        auto kf = std::make_shared<Keyframe>();

        kf->Load(ss);
        keyframes.emplace(kf->id_, kf);
    }

    LOG(INFO) << "Loaded kfs: " << keyframes.size();
    return true;
}
}  // namespace sad
