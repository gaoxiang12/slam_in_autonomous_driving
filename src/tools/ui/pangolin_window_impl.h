#ifndef FUSION_UI_PANGOLIN_WINDOW_IMPL_H
#define FUSION_UI_PANGOLIN_WINDOW_IMPL_H

// include pangolin先于pcl, 消除HAVE_OPENNI的编译警告
#include <pangolin/pangolin.h>

#include "common/point_types.h"
#include "tools/ui/pangolin_window.h"
#include "tools/ui/ui_car.h"
#include "tools/ui/ui_cloud.h"
#include "tools/ui/ui_image.h"
#include "tools/ui/ui_trajectory.h"

#include <pcl/filters/voxel_grid.h>
#include <atomic>
#include <mutex>
#include <string>
#include <thread>

namespace sad::ui {

struct UiFrame;

/**
 */
class PangolinWindowImpl {
   public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

    PangolinWindowImpl() = default;
    ~PangolinWindowImpl() = default;

    PangolinWindowImpl(const PangolinWindowImpl &) = delete;
    PangolinWindowImpl &operator=(const PangolinWindowImpl &) = delete;
    PangolinWindowImpl(PangolinWindowImpl &&) = delete;
    PangolinWindowImpl &operator=(PangolinWindowImpl &&) = delete;

    /// 初始化，创建各种点云、小车实体
    bool Init();

    /// 注销
    bool DeInit();

    /// 渲染所有信息
    void Render();

   public:
    /// 后台渲染线程
    std::thread render_thread_;

    /// 一些辅助的锁和原子变量
    std::mutex mtx_map_cloud_;
    std::mutex mtx_current_scan_;
    std::mutex mtx_nav_state_;
    std::mutex mtx_lidarloc_pose_;
    std::mutex mtx_pgo_pose_;
    std::mutex mtx_gps_pose_;

    std::atomic<bool> exit_flag_;

    std::atomic<bool> cloud_global_need_update_;
    std::atomic<bool> kf_result_need_update_;
    std::atomic<bool> current_scan_need_update_;
    std::atomic<bool> lidarloc_need_update_;
    std::atomic<bool> pgoloc_need_update_;
    std::atomic<bool> gps_need_update_;

    CloudPtr current_scan_ = nullptr;  // 当前scan
    SE3 current_pose_;                 // 当前scan对应的pose

    // 地图点云
    std::map<Vec2i, CloudPtr, less_vec<2>> cloud_global_map_;

    /// gps
    SE3 gps_pose_;

    /// 滤波器状态
    SE3 pose_;
    Vec3d vel_;
    Vec3d bias_acc_;
    Vec3d bias_gyr_;
    Vec3d grav_;

    SE3 T_imu_lidar_;
    int max_size_of_current_scan_ = 2000;  // 当前扫描数据保留多少个

    //////////////////////////////// 以下和render相关 ///////////////////////////
   private:
    /// 创建OpenGL Buffers
    void AllocateBuffer();
    void ReleaseBuffer();

    void CreateDisplayLayout();

    void DrawAll();  // 作图：画定位窗口
    void SwitchBetweenPgoAndLoc();

    /// 渲染点云，调用各种Update函数
    void RenderClouds();
    bool UpdateGps();
    bool UpdateGlobalMap();
    bool UpdateState();
    // bool UpdateLidarLoc();
    // bool UpdatePgoLoc();
    bool UpdateCurrentScan();

    void RenderScoreImage();
    void RenderPgoImage();
    void RenderGlobalPoseStatusAndLabels();
    void RenderPlot();

   private:
    /// 窗口layout相关
    int win_width_ = 1920;
    int win_height_ = 1080;
    static constexpr float cam_focus_ = 5000;
    static constexpr float cam_z_near_ = 1.0;
    static constexpr float cam_z_far_ = 1e10;
    static constexpr int menu_width_ = 200;
    const std::string win_name_ = "SAD.UI";
    const std::string dis_main_name_ = "main";
    const std::string dis_3d_name_ = "Cam 3D";
    const std::string dis_3d_main_name_ = "Cam 3D Main";  // main
    const std::string dis_plot_name_ = "Plot";
    const std::string dis_imgs_name = "Images";

    bool show_trajectory_ = true;
    bool img_y_up_ = true;       // 切换图像中Y轴的朝向。opengl: y朝上，opencv中：y朝下。
    bool following_loc_ = true;  // 相机是否追踪定位结果
    bool show_dr_traj_ = false;  // 是否在主窗口显示lio的轨迹
    bool show_gps_traj_ = true;  // 是否在主窗口显示gps的轨迹
    bool reset_dr_traj_ = true;  // reset之后置为false, 初值设为true    为true时 使用PGO的pose做起点，使用dr进行递推

    // text
    pangolin::GlText gltext_gnss_;
    pangolin::GlText gltext_lidar_;
    pangolin::GlText gltext_lidar_gnss_;
    pangolin::GlText gltext_other_;
    pangolin::GlText gltext_label_global_;
    pangolin::GlText gltext_label_matchloc_;  // 激光与xcm匹配定位 或者 视觉与HD匹配定位
    pangolin::GlText gltext_label_pgoloc_;
    pangolin::GlText gltext_label_dr_loc_;
    pangolin::GlText gltext_label_gps_;
    pangolin::GlText gltext_bag_name_;
    pangolin::GlText gltext_sensor_timeout_;

    // camera
    pangolin::OpenGlRenderState s_cam_main_;

    /// cloud rendering
    ui::UiCar car_{Vec3f(1.0, 1.0, 1.0)};                                      // 白色车
    std::map<Vec2i, std::shared_ptr<ui::UiCloud>, less_vec<2>> cloud_map_ui_;  // 用来渲染的点云地图
    std::shared_ptr<ui::UiCloud> current_scan_ui_;                             // current scan
    std::deque<std::shared_ptr<ui::UiCloud>> scans_;                           // current scan 保留的队列

    /// ui绘制中使用的一些中间变量
    // PgoResult pgo_result_ui_;             // pgo融合定位结果
    // LocResult lidarloc_result_ui_;        // 激光雷达定位结果
    SE3 T_map_odom_for_lio_traj_ui_;      // 用于显示lio的轨迹
    SE3 T_map_baselink_for_lio_traj_ui_;  // 用于显示lio的轨迹

    //  trajectory
    std::shared_ptr<ui::UiTrajectory> traj_lidarloc_ui_ = nullptr;
    std::shared_ptr<ui::UiTrajectory> traj_gps_ui_ = nullptr;

    // 滤波器状态相关 Data logger object
    pangolin::DataLog log_pos_;           // odom frame
    pangolin::DataLog log_rpy_;           // odom frame
    pangolin::DataLog log_vel_;           // odom frame下的速度
    pangolin::DataLog log_vel_baselink_;  // baselink frame下的速度
    pangolin::DataLog log_bias_acc_;      //
    pangolin::DataLog log_bias_gyr_;      //
    pangolin::DataLog log_grav_;          // odom frame
    pangolin::DataLog log_confidence_;    // 雷达/视觉定位的置信度
    pangolin::DataLog log_yaw_;           // 航向角

    std::unique_ptr<pangolin::Plotter> plotter_pos_ = nullptr;
    std::unique_ptr<pangolin::Plotter> plotter_rpy_ = nullptr;
    std::unique_ptr<pangolin::Plotter> plotter_vel_ = nullptr;
    std::unique_ptr<pangolin::Plotter> plotter_vel_baselink_ = nullptr;
    std::unique_ptr<pangolin::Plotter> plotter_bias_acc_ = nullptr;
    std::unique_ptr<pangolin::Plotter> plotter_bias_gyr_ = nullptr;
    std::unique_ptr<pangolin::Plotter> plotter_grav_ = nullptr;
    std::unique_ptr<pangolin::Plotter> plotter_confidence_ = nullptr;
    std::unique_ptr<pangolin::Plotter> plotter_yaw_ = nullptr;

    //////////////////////////////// 以上和render相关 ///////////////////////////

    /// pcl点中的颜色：白色
    static constexpr uint32_t color_white_ = 255 << 24 | 255 << 16 | 255 << 8 | 255;
    /// 合成pcl点中的颜色
    static inline uint32_t ComposeColor(uint32_t b, uint32_t g, uint32_t r, uint32_t a = 255) {
        return (a << 24 | r << 16 | g << 8 | b);
    }
};

}  // namespace sad::ui

#endif
