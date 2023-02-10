#include "tools/ui/pangolin_window.h"
#include "tools/ui/pangolin_window_impl.h"

#include <glog/logging.h>
namespace sad::ui {

PangolinWindow::PangolinWindow() { impl_ = std::make_shared<PangolinWindowImpl>(); }
PangolinWindow::~PangolinWindow() {
    LOG(INFO) << "pangolin window is deallocated.";
    Quit();
}

bool PangolinWindow::Init() {
    impl_->cloud_global_need_update_.store(false);
    impl_->kf_result_need_update_.store(false);
    impl_->lidarloc_need_update_.store(false);
    impl_->pgoloc_need_update_.store(false);
    impl_->gps_need_update_.store(false);
    impl_->current_scan_need_update_.store(false);

    bool inited = impl_->Init();
    if (inited) {
        impl_->render_thread_ = std::thread([this]() { impl_->Render(); });
    }
    return inited;
}

void PangolinWindow::Quit() {
    if (impl_->render_thread_.joinable()) {
        impl_->exit_flag_.store(true);
        // common::options::lio::flg_exit = true;
        impl_->render_thread_.join();
    }
    impl_->DeInit();
}

void PangolinWindow::UpdatePointCloudGlobal(const std::map<Vec2i, CloudPtr, less_vec<2>>& cloud) {
    std::lock_guard<std::mutex> lock(impl_->mtx_map_cloud_);
    impl_->cloud_global_map_ = cloud;
    impl_->cloud_global_need_update_.store(true);
}

void PangolinWindow::UpdateNavState(const NavStated& state) {
    std::unique_lock<std::mutex> lock_lio_res(impl_->mtx_nav_state_);

    impl_->pose_ = SE3(state.R_, state.p_);
    impl_->vel_ = state.v_;
    impl_->bias_acc_ = state.ba_;
    impl_->bias_gyr_ = state.bg_;

    impl_->kf_result_need_update_.store(true);
}

void PangolinWindow::UpdateScan(CloudPtr cloud, const SE3& pose) {
    std::lock_guard<std::mutex> lock(impl_->mtx_current_scan_);
    *impl_->current_scan_ = *cloud;  // need deep copy
    impl_->current_pose_ = pose;
    impl_->current_scan_need_update_.store(true);
}

void PangolinWindow::SetCurrentScanSize(int current_scan_size) { impl_->max_size_of_current_scan_ = current_scan_size; }

void PangolinWindow::UpdateGps(const GNSS& gps) {
    std::lock_guard<std::mutex> lock(impl_->mtx_gps_pose_);
    impl_->gps_pose_ = gps.utm_pose_;
    impl_->gps_need_update_.store(true);
}

void PangolinWindow::SetTImuLidar(const SE3& T_imu_lidar) { impl_->T_imu_lidar_ = T_imu_lidar; }

bool PangolinWindow::ShouldQuit() { return pangolin::ShouldQuit(); }

}  // namespace sad::ui
