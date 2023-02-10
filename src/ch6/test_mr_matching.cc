//
// Created by xiang on 2022/3/15.
//
#include <gflags/gflags.h>
#include <glog/logging.h>
#include <fstream>
#include <opencv2/highgui.hpp>

#include "ch6/frame.h"
#include "ch6/lidar_2d_utils.h"
#include "ch6/multi_resolution_likelihood_field.h"

/// 测试多分辨率的匹配

int main(int argc, char** argv) {
    google::InitGoogleLogging(argv[0]);
    FLAGS_stderrthreshold = google::INFO;
    FLAGS_colorlogtostderr = true;
    google::ParseCommandLineFlags(&argc, &argv, true);

    std::ifstream fin("./data/ch6/loops.txt");
    int loop_id = 0;

    while (!fin.eof()) {
        int frame_id, submap_id;
        double submap_center_x, submap_center_y, theta;
        if (fin.peek() == fin.eof()) {
            break;
        }

        fin >> frame_id >> submap_id >> submap_center_x >> submap_center_y >> theta;
        loop_id++;

        sad::MRLikelihoodField mr_field;
        Vec2d center(submap_center_x, submap_center_y);
        SE2 pose_submap(SO2::exp(theta), center);

        mr_field.SetPose(pose_submap);
        cv::Mat occu_map = cv::imread("./data/ch6/submap_" + std::to_string(submap_id) + ".png", cv::IMREAD_GRAYSCALE);
        cv::Mat occu_map_color =
            cv::imread("./data/ch6/submap_" + std::to_string(submap_id) + ".png", cv::IMREAD_COLOR);
        mr_field.SetFieldImageFromOccuMap(occu_map);

        sad::Frame frame;
        frame.Load("./data/ch6/frame_" + std::to_string(frame_id) + ".txt");
        mr_field.SetSourceScan(frame.scan_);

        LOG(INFO) << "testing frame " << frame.id_ << " with " << submap_id;

        auto init_pose = frame.pose_;
        auto frame_pose_in_submap = pose_submap.inverse() * frame.pose_;
        bool align_success = mr_field.AlignG2O(frame_pose_in_submap);

        if (align_success) {
            frame.pose_ = pose_submap * frame_pose_in_submap;
            auto images = mr_field.GetFieldImage();
            for (int i = 0; i < images.size(); ++i) {
                /// 初始pose 以红色显示
                sad::Visualize2DScan(frame.scan_, init_pose, images[i], Vec3b(0, 0, 255), images[i].rows,
                                     mr_field.Resolution(i), pose_submap);
                /// 配准后pose 以绿色显示
                sad::Visualize2DScan(frame.scan_, frame.pose_, images[i], Vec3b(0, 255, 0), images[i].rows,
                                     mr_field.Resolution(i), pose_submap);
                cv::imshow("level " + std::to_string(i), images[i]);
            }

            sad::Visualize2DScan(frame.scan_, init_pose, occu_map_color, Vec3b(0, 0, 255), occu_map_color.rows, 20.0,
                                 pose_submap);
            sad::Visualize2DScan(frame.scan_, frame.pose_, occu_map_color, Vec3b(0, 255, 0), occu_map_color.rows, 20.0,
                                 pose_submap);
            cv::imshow("occupancy", occu_map_color);
            cv::waitKey();
        }
    }

    return 0;
}