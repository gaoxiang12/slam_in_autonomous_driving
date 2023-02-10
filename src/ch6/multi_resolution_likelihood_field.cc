//
// Created by xiang on 2022/4/7.
//

#include "multi_resolution_likelihood_field.h"
#include "ch6/g2o_types.h"

#include <g2o/core/base_unary_edge.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/core/robust_kernel.h>
#include <g2o/core/robust_kernel_impl.h>
#include <g2o/core/sparse_optimizer.h>
#include <g2o/solvers/cholmod/linear_solver_cholmod.h>

#include <glog/logging.h>

namespace sad {

void MRLikelihoodField::BuildModel() {
    const int range = 20;  // 生成多少个像素的模板

    /// 生成模板金字塔图像
    field_ = {
        cv::Mat(125, 125, CV_32F, 30.0),
        cv::Mat(250, 250, CV_32F, 30.0),
        cv::Mat(500, 500, CV_32F, 30.0),
        cv::Mat(1000, 1000, CV_32F, 30.0),
    };

    for (int x = -range; x <= range; ++x) {
        for (int y = -range; y <= range; ++y) {
            model_.emplace_back(x, y, std::sqrt((x * x) + (y * y)));
        }
    }
}

void MRLikelihoodField::SetFieldImageFromOccuMap(const cv::Mat& occu_map) {
    const int boarder = 25;
    for (int x = boarder; x < occu_map.cols - boarder; ++x) {
        for (int y = boarder; y < occu_map.rows - boarder; ++y) {
            if (occu_map.at<uchar>(y, x) < 127) {
                // 在该点生成一个model，在每个level中都填入
                for (int l = 0; l < levels_; ++l) {
                    for (auto& model_pt : model_) {
                        int xx = int(x * ratios_[l] + model_pt.dx_);
                        int yy = int(y * ratios_[l] + model_pt.dy_);
                        if (xx >= 0 && xx < field_[l].cols && yy >= 0 && yy < field_[l].rows &&
                            field_[l].at<float>(yy, xx) > model_pt.residual_) {
                            field_[l].at<float>(yy, xx) = model_pt.residual_;
                        }
                    }
                }
            }
        }
    }
}

bool MRLikelihoodField::AlignG2O(SE2& init_pose) {
    num_inliers_.clear();
    inlier_ratio_.clear();

    for (int l = 0; l < levels_; ++l) {
        if (!AlignInLevel(l, init_pose)) {
            return false;
        }
    }

    /// 成功匹配的话，打印一些信息
    for (int l = 0; l < levels_; ++l) {
        LOG(INFO) << "level " << l << " inliers: " << num_inliers_[l] << ", ratio: " << inlier_ratio_[l];
    }

    return true;
}

std::vector<cv::Mat> MRLikelihoodField::GetFieldImage() {
    std::vector<cv::Mat> images;
    for (int l = 0; l < levels_; ++l) {
        cv::Mat img(field_[l].rows, field_[l].cols, CV_8UC3);
        for (int x = 0; x < field_[l].cols; ++x) {
            for (int y = 0; y < field_[l].rows; ++y) {
                float r = field_[l].at<float>(y, x) * 255.0 / 30.0;
                img.at<cv::Vec3b>(y, x) = cv::Vec3b(uchar(r), uchar(r), uchar(r));
            }
        }

        images.emplace_back(img);
    }

    return images;
}

bool MRLikelihoodField::AlignInLevel(int level, SE2& init_pose) {
    using BlockSolverType = g2o::BlockSolver<g2o::BlockSolverTraits<3, 1>>;
    using LinearSolverType = g2o::LinearSolverCholmod<BlockSolverType::PoseMatrixType>;
    auto* solver = new g2o::OptimizationAlgorithmLevenberg(
        g2o::make_unique<BlockSolverType>(g2o::make_unique<LinearSolverType>()));
    g2o::SparseOptimizer optimizer;
    optimizer.setAlgorithm(solver);

    auto* v = new VertexSE2();
    v->setId(0);
    v->setEstimate(init_pose);
    optimizer.addVertex(v);

    const double range_th = 15.0;  // 不考虑太远的scan，不准
    const double rk_delta[] = {0.2, 0.3, 0.6, 0.8};

    std::vector<EdgeSE2LikelihoodFiled*> edges;

    // 遍历source
    for (size_t i = 0; i < source_->ranges.size(); ++i) {
        float r = source_->ranges[i];
        if (r < source_->range_min || r > source_->range_max) {
            continue;
        }

        if (r > range_th) {
            continue;
        }

        float angle = source_->angle_min + i * source_->angle_increment;
        if (angle < source_->angle_min + 30 * M_PI / 180.0 || angle > source_->angle_max - 30 * M_PI / 180.0) {
            continue;
        }

        auto e = new EdgeSE2LikelihoodFiled(field_[level], r, angle, resolution_[level]);
        e->setVertex(0, v);

        if (e->IsOutSide()) {
            delete e;
            continue;
        }

        e->setInformation(Eigen::Matrix<double, 1, 1>::Identity());
        auto rk = new g2o::RobustKernelHuber;
        rk->setDelta(rk_delta[level]);
        e->setRobustKernel(rk);
        optimizer.addEdge(e);

        edges.emplace_back(e);
    }

    if (edges.empty()) {
        return false;
    }

    optimizer.setVerbose(false);
    optimizer.initializeOptimization();
    optimizer.optimize(10);

    /// 计算edges中有多少inlier
    int num_inliers =
        std::accumulate(edges.begin(), edges.end(), 0, [&rk_delta, level](int num, EdgeSE2LikelihoodFiled* e) {
            if (e->level() == 0 && e->chi2() < rk_delta[level]) {
                return num + 1;
            }
            return num;
        });

    std::vector<double> chi2(edges.size());
    for (int i = 0; i < edges.size(); ++i) {
        chi2[i] = edges[i]->chi2();
    }

    std::sort(chi2.begin(), chi2.end());

    /// 要求inlier比例超过一定值
    /// 这个比较微妙，因为激光不是360的，这里大多数时候只能部分匹配上
    const float inlier_ratio_th = 0.4;
    float inlier_ratio = float(num_inliers) / edges.size();

    num_inliers_.emplace_back(num_inliers);
    inlier_ratio_.emplace_back(inlier_ratio);

    if (num_inliers > 100 && inlier_ratio > inlier_ratio_th) {
        init_pose = v->estimate();
        return true;
    } else {
        // LOG(INFO) << "rejected because ratio is not enough: " << inlier_ratio;
        return false;
    }
}

}  // namespace sad