//
// Created by xiang on 22-12-6.
//

#ifndef SLAM_IN_AUTO_DRIVING_FRONTEND_H
#define SLAM_IN_AUTO_DRIVING_FRONTEND_H

#include <map>
#include <memory>
#include <string>

#include "common/gnss.h"
#include "common/nav_state.h"
#include "keyframe.h"

namespace sad {

class LioIEKF;
/**
 * 建图前端部分，将IMU和Lidar点云交给LIO处理，将RTK解析为rtk_pose
 */
class Frontend {
   public:
    struct Options {};

    // 从yaml文件中读取数据路径
    explicit Frontend(const std::string& config_yaml);

    // 初始化，创建LIO对象，检查数据存在性
    bool Init();

    /// 运行前端
    void Run();

   private:
    /// 看某个state是否能提取到RTK pose
    void ExtractKeyFrame(const NavStated& state);

    /// 确定某个关键帧的GPS pose
    void FindGPSPose(std::shared_ptr<Keyframe> kf);

    /// 保存各个关键帧位姿，点云在生成时已经保存
    void SaveKeyframes();

    /// 提取RTK地图原点并移除此原点
    void RemoveMapOrigin();

    // 数据
    std::shared_ptr<Keyframe> last_kf_ = nullptr;            // 最近关键帧
    std::map<IdType, std::shared_ptr<Keyframe>> keyframes_;  // 抽取的关键帧
    std::shared_ptr<LioIEKF> lio_ = nullptr;                 // LIO
    std::string config_yaml_;                                // 配置文件路径

    std::map<double, GNSSPtr> gnss_;  // GNSS 数据
    IdType kf_id_ = 0;                // 最新关键帧ID
    Vec3d map_origin_ = Vec3d::Zero();

    // 参数和配置
    std::string bag_path_;         // 数据包路径
    std::string lio_yaml_;         // LIO 配置YAML
    double kf_dis_th_ = 1.0;       // 关键帧距离阈值
    double kf_ang_th_deg_ = 10.0;  // 关键帧角度阈值（度）
};

}  // namespace sad

#endif  // SLAM_IN_AUTO_DRIVING_FRONTEND_H
