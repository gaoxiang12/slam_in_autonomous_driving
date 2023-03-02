//
// Created by xiang on 2021/8/19.
//
#include <gflags/gflags.h>
#include <glog/logging.h>
#include <gtest/gtest.h>

#include <pcl/io/pcd_io.h>
#include <pcl/search/kdtree.h>

#include "ch5/bfnn.h"
#include "ch5/gridnn.hpp"
#include "ch5/kdtree.h"
#include "ch5/octo_tree.h"
#include "common/point_cloud_utils.h"
#include "common/point_types.h"
#include "common/sys_utils.h"

DEFINE_string(first_scan_path, "./data/ch5/first.pcd", "第一个点云路径");
DEFINE_string(second_scan_path, "./data/ch5/second.pcd", "第二个点云路径");
DEFINE_double(ANN_alpha, 1.0, "AAN的比例因子");

TEST(CH5_TEST, BFNN) {
    sad::CloudPtr first(new sad::PointCloudType), second(new sad::PointCloudType);
    pcl::io::loadPCDFile(FLAGS_first_scan_path, *first);
    pcl::io::loadPCDFile(FLAGS_second_scan_path, *second);

    if (first->empty() || second->empty()) {
        LOG(ERROR) << "cannot load cloud";
        FAIL();
    }

    // voxel grid 至 0.05
    sad::VoxelGrid(first);
    sad::VoxelGrid(second);

    LOG(INFO) << "points: " << first->size() << ", " << second->size();

    // 评价单线程和多线程版本的暴力匹配
    sad::evaluate_and_call(
        [&first, &second]() {
            std::vector<std::pair<size_t, size_t>> matches;
            sad::bfnn_cloud(first, second, matches);
        },
        "暴力匹配（单线程）", 5);
    sad::evaluate_and_call(
        [&first, &second]() {
            std::vector<std::pair<size_t, size_t>> matches;
            sad::bfnn_cloud_mt(first, second, matches);
        },
        "暴力匹配（多线程）", 5);

    SUCCEED();
}

/**
 * 评测最近邻的正确性
 * @param truth 真值
 * @param esti  估计
 */
void EvaluateMatches(const std::vector<std::pair<size_t, size_t>>& truth,
                     const std::vector<std::pair<size_t, size_t>>& esti) {
    int fp = 0;  // false-positive，esti存在但truth中不存在
    int fn = 0;  // false-negative, truth存在但esti不存在

    LOG(INFO) << "truth: " << truth.size() << ", esti: " << esti.size();

    /// 检查某个匹配在另一个容器中存不存在
    auto exist = [](const std::pair<size_t, size_t>& data, const std::vector<std::pair<size_t, size_t>>& vec) -> bool {
        return std::find(vec.begin(), vec.end(), data) != vec.end();
    };

    int effective_esti = 0;
    for (const auto& d : esti) {
        if (d.first != sad::math::kINVALID_ID && d.second != sad::math::kINVALID_ID) {
            effective_esti++;

            if (!exist(d, truth)) {
                fp++;
            }
        }
    }

    for (const auto& d : truth) {
        if (!exist(d, esti)) {
            fn++;
        }
    }

    float precision = 1.0 - float(fp) / effective_esti;
    float recall = 1.0 - float(fn) / truth.size();
    LOG(INFO) << "precision: " << precision << ", recall: " << recall << ", fp: " << fp << ", fn: " << fn;
}

TEST(CH5_TEST, GRID_NN) {
    sad::CloudPtr first(new sad::PointCloudType), second(new sad::PointCloudType);
    pcl::io::loadPCDFile(FLAGS_first_scan_path, *first);
    pcl::io::loadPCDFile(FLAGS_second_scan_path, *second);

    if (first->empty() || second->empty()) {
        LOG(ERROR) << "cannot load cloud";
        FAIL();
    }

    // voxel grid 至 0.05
    sad::VoxelGrid(first);
    sad::VoxelGrid(second);

    LOG(INFO) << "points: " << first->size() << ", " << second->size();

    std::vector<std::pair<size_t, size_t>> truth_matches;
    sad::bfnn_cloud(first, second, truth_matches);

    // 对比不同种类的grid
    sad::GridNN<2> grid0(0.1, sad::GridNN<2>::NearbyType::CENTER), grid4(0.1, sad::GridNN<2>::NearbyType::NEARBY4),
        grid8(0.1, sad::GridNN<2>::NearbyType::NEARBY8);
    sad::GridNN<3> grid3(0.1, sad::GridNN<3>::NearbyType::NEARBY6);

    grid0.SetPointCloud(first);
    grid4.SetPointCloud(first);
    grid8.SetPointCloud(first);
    grid3.SetPointCloud(first);

    // 评价各种版本的Grid NN
    // sorry没有C17的template lambda... 下面必须写的啰嗦一些
    LOG(INFO) << "===================";
    std::vector<std::pair<size_t, size_t>> matches;
    sad::evaluate_and_call(
        [&first, &second, &grid0, &matches]() { grid0.GetClosestPointForCloud(first, second, matches); },
        "Grid0 单线程", 10);
    EvaluateMatches(truth_matches, matches);

    LOG(INFO) << "===================";
    sad::evaluate_and_call(
        [&first, &second, &grid0, &matches]() { grid0.GetClosestPointForCloudMT(first, second, matches); },
        "Grid0 多线程", 10);
    EvaluateMatches(truth_matches, matches);

    LOG(INFO) << "===================";
    sad::evaluate_and_call(
        [&first, &second, &grid4, &matches]() { grid4.GetClosestPointForCloud(first, second, matches); },
        "Grid4 单线程", 10);
    EvaluateMatches(truth_matches, matches);

    LOG(INFO) << "===================";
    sad::evaluate_and_call(
        [&first, &second, &grid4, &matches]() { grid4.GetClosestPointForCloudMT(first, second, matches); },
        "Grid4 多线程", 10);
    EvaluateMatches(truth_matches, matches);

    LOG(INFO) << "===================";
    sad::evaluate_and_call(
        [&first, &second, &grid8, &matches]() { grid8.GetClosestPointForCloud(first, second, matches); },
        "Grid8 单线程", 10);
    EvaluateMatches(truth_matches, matches);

    LOG(INFO) << "===================";
    sad::evaluate_and_call(
        [&first, &second, &grid8, &matches]() { grid8.GetClosestPointForCloudMT(first, second, matches); },
        "Grid8 多线程", 10);
    EvaluateMatches(truth_matches, matches);

    LOG(INFO) << "===================";
    sad::evaluate_and_call(
        [&first, &second, &grid3, &matches]() { grid3.GetClosestPointForCloud(first, second, matches); },
        "Grid 3D 单线程", 10);
    EvaluateMatches(truth_matches, matches);

    LOG(INFO) << "===================";
    sad::evaluate_and_call(
        [&first, &second, &grid3, &matches]() { grid3.GetClosestPointForCloudMT(first, second, matches); },
        "Grid 3D 多线程", 10);
    EvaluateMatches(truth_matches, matches);

    SUCCEED();
}

TEST(CH5_TEST, KDTREE_BASICS) {
    sad::CloudPtr cloud(new sad::PointCloudType);
    sad::PointType p1, p2, p3, p4;
    p1.x = 0;
    p1.y = 0;
    p1.z = 0;

    p2.x = 1;
    p2.y = 0;
    p2.z = 0;

    p3.x = 0;
    p3.y = 1;
    p3.z = 0;

    p4.x = 1;
    p4.y = 1;
    p4.z = 0;

    cloud->points.push_back(p1);
    cloud->points.push_back(p2);
    cloud->points.push_back(p3);
    cloud->points.push_back(p4);

    sad::KdTree kdtree;
    kdtree.BuildTree(cloud);
    kdtree.PrintAll();

    SUCCEED();
}

TEST(CH5_TEST, KDTREE_KNN) {
    sad::CloudPtr first(new sad::PointCloudType), second(new sad::PointCloudType);
    pcl::io::loadPCDFile(FLAGS_first_scan_path, *first);
    pcl::io::loadPCDFile(FLAGS_second_scan_path, *second);

    if (first->empty() || second->empty()) {
        LOG(ERROR) << "cannot load cloud";
        FAIL();
    }

    // voxel grid 至 0.05
    sad::VoxelGrid(first);
    sad::VoxelGrid(second);

    sad::KdTree kdtree;
    sad::evaluate_and_call([&first, &kdtree]() { kdtree.BuildTree(first); }, "Kd Tree build", 1);

    kdtree.SetEnableANN(true, FLAGS_ANN_alpha);

    LOG(INFO) << "Kd tree leaves: " << kdtree.size() << ", points: " << first->size();

    // 比较 bfnn
    std::vector<std::pair<size_t, size_t>> true_matches;
    sad::bfnn_cloud_mt_k(first, second, true_matches);

    // 对第2个点云执行knn
    std::vector<std::pair<size_t, size_t>> matches;
    sad::evaluate_and_call([&first, &second, &kdtree, &matches]() { kdtree.GetClosestPointMT(second, matches, 5); },
                           "Kd Tree 5NN 多线程", 1);
    EvaluateMatches(true_matches, matches);

    LOG(INFO) << "building kdtree pcl";
    // 对比PCL
    pcl::search::KdTree<sad::PointType> kdtree_pcl;
    sad::evaluate_and_call([&first, &kdtree_pcl]() { kdtree_pcl.setInputCloud(first); }, "Kd Tree build", 1);

    LOG(INFO) << "searching pcl";
    matches.clear();
    std::vector<int> search_indices(second->size());
    for (int i = 0; i < second->points.size(); i++) {
        search_indices[i] = i;
    }

    std::vector<std::vector<int>> result_index;
    std::vector<std::vector<float>> result_distance;
    sad::evaluate_and_call(
        [&]() { kdtree_pcl.nearestKSearch(*second, search_indices, 5, result_index, result_distance); },
        "Kd Tree 5NN in PCL", 1);
    for (int i = 0; i < second->points.size(); i++) {
        for (int j = 0; j < result_index[i].size(); ++j) {
            int m = result_index[i][j];
            double d = result_distance[i][j];
            matches.push_back({m, i});
        }
    }
    EvaluateMatches(true_matches, matches);

    LOG(INFO) << "done.";

    SUCCEED();
}

TEST(CH5_TEST, OCTREE_BASICS) {
    sad::CloudPtr cloud(new sad::PointCloudType);
    sad::PointType p1, p2, p3, p4;
    p1.x = 0;
    p1.y = 0;
    p1.z = 0;

    p2.x = 1;
    p2.y = 0;
    p2.z = 0;

    p3.x = 0;
    p3.y = 1;
    p3.z = 0;

    p4.x = 1;
    p4.y = 1;
    p4.z = 0;

    cloud->points.push_back(p1);
    cloud->points.push_back(p2);
    cloud->points.push_back(p3);
    cloud->points.push_back(p4);

    sad::OctoTree octree;
    octree.BuildTree(cloud);
    octree.SetApproximate(false);
    LOG(INFO) << "Octo tree leaves: " << octree.size() << ", points: " << cloud->size();

    SUCCEED();
}

TEST(CH5_TEST, OCTREE_KNN) {
    sad::CloudPtr first(new sad::PointCloudType), second(new sad::PointCloudType);
    pcl::io::loadPCDFile(FLAGS_first_scan_path, *first);
    pcl::io::loadPCDFile(FLAGS_second_scan_path, *second);

    if (first->empty() || second->empty()) {
        LOG(ERROR) << "cannot load cloud";
        FAIL();
    }

    // voxel grid 至 0.05
    sad::VoxelGrid(first);
    sad::VoxelGrid(second);

    sad::OctoTree octree;
    sad::evaluate_and_call([&first, &octree]() { octree.BuildTree(first); }, "Octo Tree build", 1);

    octree.SetApproximate(true, FLAGS_ANN_alpha);
    LOG(INFO) << "Octo tree leaves: " << octree.size() << ", points: " << first->size();

    /// 测试KNN
    LOG(INFO) << "testing knn";
    std::vector<std::pair<size_t, size_t>> matches;
    sad::evaluate_and_call([&first, &second, &octree, &matches]() { octree.GetClosestPointMT(second, matches, 5); },
                           "Octo Tree 5NN 多线程", 1);

    LOG(INFO) << "comparing with bfnn";
    /// 比较真值
    std::vector<std::pair<size_t, size_t>> true_matches;
    sad::bfnn_cloud_mt_k(first, second, true_matches);
    EvaluateMatches(true_matches, matches);

    LOG(INFO) << "done.";

    SUCCEED();
}

int main(int argc, char** argv) {
    google::InitGoogleLogging(argv[0]);
    FLAGS_stderrthreshold = google::INFO;
    FLAGS_colorlogtostderr = true;

    testing::InitGoogleTest(&argc, argv);
    google::ParseCommandLineFlags(&argc, &argv, true);
    return RUN_ALL_TESTS();
}
