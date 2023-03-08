//
// Created by xiang on 22-12-6.
//

#include "frontend.h"
#include "ch8/lio-iekf/lio_iekf.h"
#include "common/io_utils.h"

#include <yaml-cpp/yaml.h>

namespace sad {

Frontend::Frontend(const std::string& config_yaml) { config_yaml_ = config_yaml; }

bool Frontend::Init() {
    LOG(INFO) << "load yaml from " << config_yaml_;
    auto yaml = YAML::LoadFile(config_yaml_);
    try {
        auto n = yaml["bag_path"];
        LOG(INFO) << Dump(n);
        bag_path_ = yaml["bag_path"].as<std::string>();
        lio_yaml_ = yaml["lio_yaml"].as<std::string>();
    } catch (...) {
        LOG(ERROR) << "failed to parse yaml";
        return false;
    }

    system("rm -rf ./data/ch9/*.pcd");
    system("rm -rf ./data/ch9/keyframes.txt");

    LioIEKF::Options options;
    options.with_ui_ = false;  // 跑建图不需要打开前端UI
    lio_ = std::make_shared<LioIEKF>(options);
    lio_->Init(lio_yaml_);
    return true;
}

void Frontend::Run() {
    sad::RosbagIO rosbag_io(bag_path_, DatasetType::NCLT);

    // 先提取RTK pose，注意NCLT只有平移部分
    rosbag_io
        .AddAutoRTKHandle([this](GNSSPtr gnss) {
            gnss_.emplace(gnss->unix_time_, gnss);
            return true;
        })
        .Go();
    rosbag_io.CleanProcessFunc();  // 不再需要处理RTK

    RemoveMapOrigin();

    // 再运行LIO
    rosbag_io
        .AddAutoPointCloudHandle([&](sensor_msgs::PointCloud2::Ptr cloud) -> bool {
            lio_->PCLCallBack(cloud);
            ExtractKeyFrame(lio_->GetCurrentState());
            return true;
        })
        .AddImuHandle([&](IMUPtr imu) {
            lio_->IMUCallBack(imu);
            return true;
        })
        .Go();
    lio_->Finish();

    // 保存运行结果
    SaveKeyframes();

    LOG(INFO) << "done.";
}

void Frontend::ExtractKeyFrame(const sad::NavStated& state) {
    if (last_kf_ == nullptr) {
        if (!lio_->GetCurrentScan()) {
            // LIO没完成初始化
            return;
        }
        // 第一个帧
        auto kf = std::make_shared<Keyframe>(state.timestamp_, kf_id_++, state.GetSE3(), lio_->GetCurrentScan());
        FindGPSPose(kf);
        kf->SaveAndUnloadScan("./data/ch9/");
        keyframes_.emplace(kf->id_, kf);
        last_kf_ = kf;
    } else {
        // 计算当前state与kf之间的相对运动阈值
        SE3 delta = last_kf_->lidar_pose_.inverse() * state.GetSE3();
        if (delta.translation().norm() > kf_dis_th_ || delta.so3().log().norm() > kf_ang_th_deg_ * math::kDEG2RAD) {
            auto kf = std::make_shared<Keyframe>(state.timestamp_, kf_id_++, state.GetSE3(), lio_->GetCurrentScan());
            FindGPSPose(kf);
            keyframes_.emplace(kf->id_, kf);
            kf->SaveAndUnloadScan("./data/ch9/");
            LOG(INFO) << "生成关键帧" << kf->id_;
            last_kf_ = kf;
        }
    }
}

void Frontend::FindGPSPose(std::shared_ptr<Keyframe> kf) {
    SE3 pose;
    GNSSPtr match;
    if (math::PoseInterp<GNSSPtr>(
            kf->timestamp_, gnss_, [](const GNSSPtr& gnss) -> SE3 { return gnss->utm_pose_; }, pose, match)) {
        kf->rtk_pose_ = pose;
        kf->rtk_valid_ = true;
    } else {
        kf->rtk_valid_ = false;
    }
}

void Frontend::SaveKeyframes() {
    std::ofstream fout("./data/ch9/keyframes.txt");
    for (auto& kfp : keyframes_) {
        kfp.second->Save(fout);
    }
    fout.close();
}

void Frontend::RemoveMapOrigin() {
    if (gnss_.empty()) {
        return;
    }

    bool origin_set = false;
    for (auto& p : gnss_) {
        if (p.second->status_ == GpsStatusType::GNSS_FIXED_SOLUTION) {
            map_origin_ = p.second->utm_pose_.translation();
            origin_set = true;

            LOG(INFO) << "map origin is set to " << map_origin_.transpose();

            auto yaml = YAML::LoadFile(config_yaml_);
            std::vector<double> ori{map_origin_[0], map_origin_[1], map_origin_[2]};
            yaml["origin"] = ori;

            std::ofstream fout(config_yaml_);
            fout << yaml;
            break;
        }
    }

    if (origin_set) {
        LOG(INFO) << "removing origin from rtk";
        for (auto& p : gnss_) {
            p.second->utm_pose_.translation() -= map_origin_;
        }
    }
}

}  // namespace sad