//
// Created by xiang on 2021/7/16.
//

#include <gflags/gflags.h>
#include <glog/logging.h>
#include <gtest/gtest.h>

#include "ch3/eskf.hpp"
#include "ch3/static_imu_init.h"
#include "ch4/imu_preintegration.h"
#include "ch4/g2o_types.h"
#include "common/g2o_types.h"
#include "common/io_utils.h"

#include <g2o/core/block_solver.h>
#include <g2o/core/optimization_algorithm_gauss_newton.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/core/robust_kernel.h>
#include <g2o/solvers/eigen/linear_solver_eigen.h>

DEFINE_string(txt_path, "./data/ch3/10.txt", "数据文件路径");
DEFINE_double(antenna_angle, 12.06, "RTK天线安装偏角（角度）");
DEFINE_double(antenna_pox_x, -0.17, "RTK天线安装偏移X");
DEFINE_double(antenna_pox_y, -0.20, "RTK天线安装偏移Y");
DEFINE_bool(with_ui, true, "是否显示图形界面");

TEST(PREINTEGRATION_TEST, ROTATION_TEST) {
    // 测试在恒定角速度运转下的预积分情况
    double imu_time_span = 0.01;       // IMU测量间隔
    Vec3d constant_omega(0, 0, M_PI);  // 角速度为180度/s，转1秒应该等于转180度
    Vec3d gravity(0, 0, -9.8);         // Z 向上，重力方向为负

    sad::NavStated start_status(0), end_status(1.0);
    sad::IMUPreintegration pre_integ;

    // 对比直接积分
    Sophus::SO3d R;
    Vec3d t = Vec3d::Zero();
    Vec3d v = Vec3d::Zero();

    for (int i = 1; i <= 100; ++i) {
        double time = imu_time_span * i;
        Vec3d acce = -gravity;  // 加速度计应该测量到一个向上的力
        pre_integ.Integrate(sad::IMU(time, constant_omega, acce), imu_time_span);

        sad::NavStated this_status = pre_integ.Predict(start_status, gravity);

        t = t + v * imu_time_span + 0.5 * gravity * imu_time_span * imu_time_span +
            0.5 * (R * acce) * imu_time_span * imu_time_span;
        v = v + gravity * imu_time_span + (R * acce) * imu_time_span;
        R = R * Sophus::SO3d::exp(constant_omega * imu_time_span);

        // 验证在简单情况下，直接积分和预积分结果相等
        EXPECT_NEAR(t[0], this_status.p_[0], 1e-2);
        EXPECT_NEAR(t[1], this_status.p_[1], 1e-2);
        EXPECT_NEAR(t[2], this_status.p_[2], 1e-2);

        EXPECT_NEAR(v[0], this_status.v_[0], 1e-2);
        EXPECT_NEAR(v[1], this_status.v_[1], 1e-2);
        EXPECT_NEAR(v[2], this_status.v_[2], 1e-2);

        EXPECT_NEAR(R.unit_quaternion().x(), this_status.R_.unit_quaternion().x(), 1e-4);
        EXPECT_NEAR(R.unit_quaternion().y(), this_status.R_.unit_quaternion().y(), 1e-4);
        EXPECT_NEAR(R.unit_quaternion().z(), this_status.R_.unit_quaternion().z(), 1e-4);
        EXPECT_NEAR(R.unit_quaternion().w(), this_status.R_.unit_quaternion().w(), 1e-4);
    }

    end_status = pre_integ.Predict(start_status);

    LOG(INFO) << "preinteg result: ";
    LOG(INFO) << "end rotation: \n" << end_status.R_.matrix();
    LOG(INFO) << "end trans: \n" << end_status.p_.transpose();
    LOG(INFO) << "end v: \n" << end_status.v_.transpose();

    LOG(INFO) << "direct integ result: ";
    LOG(INFO) << "end rotation: \n" << R.matrix();
    LOG(INFO) << "end trans: \n" << t.transpose();
    LOG(INFO) << "end v: \n" << v.transpose();
    SUCCEED();
}

TEST(PREINTEGRATION_TEST, ACCELERATION_TEST) {
    // 测试在恒定加速度运行下的预积分情况
    double imu_time_span = 0.01;     // IMU测量间隔
    Vec3d gravity(0, 0, -9.8);       // Z 向上，重力方向为负
    Vec3d constant_acce(0.1, 0, 0);  // x 方向上的恒定加速度

    sad::NavStated start_status(0), end_status(1.0);
    sad::IMUPreintegration pre_integ;

    // 对比直接积分
    Sophus::SO3d R;
    Vec3d t = Vec3d::Zero();
    Vec3d v = Vec3d::Zero();

    for (int i = 1; i <= 100; ++i) {
        double time = imu_time_span * i;
        Vec3d acce = constant_acce - gravity;
        pre_integ.Integrate(sad::IMU(time, Vec3d::Zero(), acce), imu_time_span);
        sad::NavStated this_status = pre_integ.Predict(start_status, gravity);

        t = t + v * imu_time_span + 0.5 * gravity * imu_time_span * imu_time_span +
            0.5 * (R * acce) * imu_time_span * imu_time_span;
        v = v + gravity * imu_time_span + (R * acce) * imu_time_span;

        // 验证在简单情况下，直接积分和预积分结果相等
        EXPECT_NEAR(t[0], this_status.p_[0], 1e-2);
        EXPECT_NEAR(t[1], this_status.p_[1], 1e-2);
        EXPECT_NEAR(t[2], this_status.p_[2], 1e-2);

        EXPECT_NEAR(v[0], this_status.v_[0], 1e-2);
        EXPECT_NEAR(v[1], this_status.v_[1], 1e-2);
        EXPECT_NEAR(v[2], this_status.v_[2], 1e-2);

        EXPECT_NEAR(R.unit_quaternion().x(), this_status.R_.unit_quaternion().x(), 1e-4);
        EXPECT_NEAR(R.unit_quaternion().y(), this_status.R_.unit_quaternion().y(), 1e-4);
        EXPECT_NEAR(R.unit_quaternion().z(), this_status.R_.unit_quaternion().z(), 1e-4);
        EXPECT_NEAR(R.unit_quaternion().w(), this_status.R_.unit_quaternion().w(), 1e-4);
    }

    end_status = pre_integ.Predict(start_status);
    LOG(INFO) << "preinteg result: ";
    LOG(INFO) << "end rotation: \n" << end_status.R_.matrix();
    LOG(INFO) << "end trans: \n" << end_status.p_.transpose();
    LOG(INFO) << "end v: \n" << end_status.v_.transpose();

    LOG(INFO) << "direct integ result: ";
    LOG(INFO) << "end rotation: \n" << R.matrix();
    LOG(INFO) << "end trans: \n" << t.transpose();
    LOG(INFO) << "end v: \n" << v.transpose();
    SUCCEED();
}

void Optimize(sad::NavStated& last_state, sad::NavStated& this_state, sad::GNSS& last_gnss, sad::GNSS& this_gnss,
              std::shared_ptr<sad::IMUPreintegration>& preinteg, const Vec3d& grav);

/// 使用ESKF的Predict, Update来验证预积分的优化过程
TEST(PREINTEGRATION_TEST, ESKF_TEST) {
    if (fLS::FLAGS_txt_path.empty()) {
        FAIL();
    }

    // 初始化器
    sad::StaticIMUInit imu_init;  // 使用默认配置
    sad::ESKFD eskf;

    sad::TxtIO io(FLAGS_txt_path);
    Vec2d antenna_pos(FLAGS_antenna_pox_x, FLAGS_antenna_pox_y);

    std::ofstream fout("./data/ch3/gins.txt");
    bool imu_inited = false, gnss_inited = false;

    /// 设置各类回调函数
    bool first_gnss_set = false;
    Vec3d origin = Vec3d::Zero();

    std::shared_ptr<sad::IMUPreintegration> preinteg = nullptr;

    sad::NavStated last_state;
    bool last_state_set = false;

    sad::GNSS last_gnss;
    bool last_gnss_set = false;

    io.SetIMUProcessFunc([&](const sad::IMU& imu) {
          /// IMU 处理函数
          if (!imu_init.InitSuccess()) {
              imu_init.AddIMU(imu);
              return;
          }

          /// 需要IMU初始化
          if (!imu_inited) {
              // 读取初始零偏，设置ESKF
              sad::ESKFD::Options options;
              // 噪声由初始化器估计
              options.gyro_var_ = sqrt(imu_init.GetCovGyro()[0]);
              options.acce_var_ = sqrt(imu_init.GetCovAcce()[0]);
              eskf.SetInitialConditions(options, imu_init.GetInitBg(), imu_init.GetInitBa(), imu_init.GetGravity());

              imu_inited = true;
              return;
          }

          if (!gnss_inited) {
              /// 等待有效的RTK数据
              return;
          }

          /// GNSS 也接收到之后，再开始进行预测
          double current_time = eskf.GetNominalState().timestamp_;
          eskf.Predict(imu);

          if (preinteg) {
              preinteg->Integrate(imu, imu.timestamp_ - current_time);

              if (last_state_set) {
                  auto pred_of_preinteg = preinteg->Predict(last_state, eskf.GetGravity());
                  auto pred_of_eskf = eskf.GetNominalState();

                  /// 这两个预测值的误差应该非常接近
                  EXPECT_NEAR((pred_of_preinteg.p_ - pred_of_eskf.p_).norm(), 0, 1e-2);
                  EXPECT_NEAR((pred_of_preinteg.R_.inverse() * pred_of_eskf.R_).log().norm(), 0, 1e-2);
                  EXPECT_NEAR((pred_of_preinteg.v_ - pred_of_eskf.v_).norm(), 0, 1e-2);
              }
          }
      })
        .SetGNSSProcessFunc([&](const sad::GNSS& gnss) {
            /// GNSS 处理函数
            if (!imu_inited) {
                return;
            }

            sad::GNSS gnss_convert = gnss;
            if (!sad::ConvertGps2UTM(gnss_convert, antenna_pos, FLAGS_antenna_angle) || !gnss_convert.heading_valid_) {
                return;
            }

            /// 去掉原点
            if (!first_gnss_set) {
                origin = gnss_convert.utm_pose_.translation();
                first_gnss_set = true;
            }
            gnss_convert.utm_pose_.translation() -= origin;

            // 要求RTK heading有效，才能合入EKF
            auto state_bef_update = eskf.GetNominalState();

            eskf.ObserveGps(gnss_convert);

            // 验证优化过程是否正确
            if (last_state_set && last_gnss_set) {
                auto update_state = eskf.GetNominalState();

                LOG(INFO) << "state before eskf update: " << state_bef_update;
                LOG(INFO) << "state after  eskf update: " << update_state;

                auto state_pred = preinteg->Predict(last_state, eskf.GetGravity());
                LOG(INFO) << "state in pred: " << state_pred;

                Optimize(last_state, update_state, last_gnss, gnss_convert, preinteg, eskf.GetGravity());
            }

            last_state = eskf.GetNominalState();
            last_state_set = true;

            // 重置预积分
            sad::IMUPreintegration::Options options;
            options.init_bg_ = last_state.bg_;
            options.init_ba_ = last_state.ba_;
            preinteg = std::make_shared<sad::IMUPreintegration>(options);

            gnss_inited = true;
            last_gnss = gnss_convert;
            last_gnss_set = true;
        })
        .SetOdomProcessFunc([&](const sad::Odom& odom) { imu_init.AddOdom(odom); })
        .Go();

    SUCCEED();
}

void Optimize(sad::NavStated& last_state, sad::NavStated& this_state, sad::GNSS& last_gnss, sad::GNSS& this_gnss,
              std::shared_ptr<sad::IMUPreintegration>& pre_integ, const Vec3d& grav) {
    assert(pre_integ != nullptr);

    if (pre_integ->dt_ < 1e-3) {
        // 未得到积分
        return;
    }

    using BlockSolverType = g2o::BlockSolverX;
    using LinearSolverType = g2o::LinearSolverEigen<BlockSolverType::PoseMatrixType>;

    auto* solver = new g2o::OptimizationAlgorithmGaussNewton(
        g2o::make_unique<BlockSolverType>(g2o::make_unique<LinearSolverType>()));
    g2o::SparseOptimizer optimizer;
    optimizer.setAlgorithm(solver);

    // 上时刻顶点， pose, v, bg, ba
    auto v0_pose = new sad::VertexPose();
    v0_pose->setId(0);
    v0_pose->setEstimate(last_state.GetSE3());
    optimizer.addVertex(v0_pose);

    auto v0_vel = new sad::VertexVelocity();
    v0_vel->setId(1);
    v0_vel->setEstimate(last_state.v_);
    optimizer.addVertex(v0_vel);

    auto v0_bg = new sad::VertexGyroBias();
    v0_bg->setId(2);
    v0_bg->setEstimate(last_state.bg_);
    optimizer.addVertex(v0_bg);

    auto v0_ba = new sad::VertexAccBias();
    v0_ba->setId(3);
    v0_ba->setEstimate(last_state.ba_);
    optimizer.addVertex(v0_ba);

    // 本时刻顶点，pose, v, bg, ba
    auto v1_pose = new sad::VertexPose();
    v1_pose->setId(4);
    v1_pose->setEstimate(this_state.GetSE3());
    optimizer.addVertex(v1_pose);

    auto v1_vel = new sad::VertexVelocity();
    v1_vel->setId(5);
    v1_vel->setEstimate(this_state.v_);
    optimizer.addVertex(v1_vel);

    auto v1_bg = new sad::VertexGyroBias();
    v1_bg->setId(6);
    v1_bg->setEstimate(this_state.bg_);
    optimizer.addVertex(v1_bg);

    auto v1_ba = new sad::VertexAccBias();
    v1_ba->setId(7);
    v1_ba->setEstimate(this_state.ba_);
    optimizer.addVertex(v1_ba);

    // 预积分边
    auto edge_inertial = new sad::EdgeInertial(pre_integ, grav);
    edge_inertial->setVertex(0, v0_pose);
    edge_inertial->setVertex(1, v0_vel);
    edge_inertial->setVertex(2, v0_bg);
    edge_inertial->setVertex(3, v0_ba);
    edge_inertial->setVertex(4, v1_pose);
    edge_inertial->setVertex(5, v1_vel);

    auto* rk = new g2o::RobustKernelHuber();
    rk->setDelta(200.0);
    edge_inertial->setRobustKernel(rk);

    optimizer.addEdge(edge_inertial);
    edge_inertial->computeError();
    LOG(INFO) << "inertial init err: " << edge_inertial->chi2();

    auto* edge_gyro_rw = new sad::EdgeGyroRW();
    edge_gyro_rw->setVertex(0, v0_bg);
    edge_gyro_rw->setVertex(1, v1_bg);
    edge_gyro_rw->setInformation(Mat3d::Identity() * 1e6);
    optimizer.addEdge(edge_gyro_rw);

    edge_gyro_rw->computeError();
    LOG(INFO) << "inertial bg rw: " << edge_gyro_rw->chi2();

    auto* edge_acc_rw = new sad::EdgeAccRW();
    edge_acc_rw->setVertex(0, v0_ba);
    edge_acc_rw->setVertex(1, v1_ba);
    edge_acc_rw->setInformation(Mat3d::Identity() * 1e6);
    optimizer.addEdge(edge_acc_rw);

    edge_acc_rw->computeError();
    LOG(INFO) << "inertial ba rw: " << edge_acc_rw->chi2();

    // GNSS边
    auto edge_gnss0 = new sad::EdgeGNSS(v0_pose, last_gnss.utm_pose_);
    edge_gnss0->setInformation(Mat6d::Identity() * 1e2);
    optimizer.addEdge(edge_gnss0);

    edge_gnss0->computeError();
    LOG(INFO) << "gnss0 init err: " << edge_gnss0->chi2();

    auto edge_gnss1 = new sad::EdgeGNSS(v1_pose, this_gnss.utm_pose_);
    edge_gnss1->setInformation(Mat6d::Identity() * 1e2);
    optimizer.addEdge(edge_gnss1);

    edge_gnss1->computeError();
    LOG(INFO) << "gnss1 init err: " << edge_gnss1->chi2();

    optimizer.setVerbose(true);
    optimizer.initializeOptimization();
    optimizer.optimize(10);

    sad::NavStated corr_state(this_state.timestamp_, v1_pose->estimate().so3(), v1_pose->estimate().translation(),
                              v1_vel->estimate(), v1_bg->estimate(), v1_ba->estimate());
    LOG(INFO) << "corr state in opt: " << corr_state;

    // 获取结果，统计各类误差
    LOG(INFO) << "chi2/error: ";
    LOG(INFO) << "preintegration: " << edge_inertial->chi2() << "/" << edge_inertial->error().transpose();
    LOG(INFO) << "gnss0: " << edge_gnss0->chi2() << ", " << edge_gnss0->error().transpose();
    LOG(INFO) << "gnss1: " << edge_gnss1->chi2() << ", " << edge_gnss1->error().transpose();
    LOG(INFO) << "bias: " << edge_gyro_rw->chi2() << "/" << edge_acc_rw->error().transpose();
}

int main(int argc, char** argv) {
    google::InitGoogleLogging(argv[0]);
    FLAGS_stderrthreshold = google::INFO;
    FLAGS_colorlogtostderr = true;

    testing::InitGoogleTest(&argc, argv);
    google::ParseCommandLineFlags(&argc, &argv, true);
    return RUN_ALL_TESTS();
}