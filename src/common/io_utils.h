//
// Created by xiang on 2021/7/20.
//

#ifndef SLAM_IN_AUTO_DRIVING_IO_UTILS_H
#define SLAM_IN_AUTO_DRIVING_IO_UTILS_H

#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <sensor_msgs/LaserScan.h>
#include <fstream>
#include <functional>
#include <utility>

#include "common/dataset_type.h"
#include "common/global_flags.h"
#include "common/gnss.h"
#include "common/imu.h"
#include "common/lidar_utils.h"
#include "common/math_utils.h"
#include "common/message_def.h"
#include "common/odom.h"
#include "livox_ros_driver/CustomMsg.h"
#include "tools/pointcloud_convert/velodyne_convertor.h"

#include "ch3/utm_convert.h"

namespace sad {

/**
 * 读取本书提供的数据文本文件，并调用回调函数
 * 数据文本文件主要提供IMU/Odom/GNSS读数
 */
class TxtIO {
   public:
    TxtIO(const std::string &file_path) : fin(file_path) {}

    /// 定义回调函数
    using IMUProcessFuncType = std::function<void(const IMU &)>;
    using OdomProcessFuncType = std::function<void(const Odom &)>;
    using GNSSProcessFuncType = std::function<void(const GNSS &)>;

    TxtIO &SetIMUProcessFunc(IMUProcessFuncType imu_proc) {
        imu_proc_ = std::move(imu_proc);
        return *this;
    }

    TxtIO &SetOdomProcessFunc(OdomProcessFuncType odom_proc) {
        odom_proc_ = std::move(odom_proc);
        return *this;
    }

    TxtIO &SetGNSSProcessFunc(GNSSProcessFuncType gnss_proc) {
        gnss_proc_ = std::move(gnss_proc);
        return *this;
    }

    // 遍历文件内容，调用回调函数
    void Go();

   private:
    std::ifstream fin;
    IMUProcessFuncType imu_proc_;
    OdomProcessFuncType odom_proc_;
    GNSSProcessFuncType gnss_proc_;
};

/**
 * ROSBAG IO
 * 指定一个包名，添加一些回调函数，就可以顺序遍历这个包
 */
class RosbagIO {
   public:
    explicit RosbagIO(std::string bag_file, DatasetType dataset_type = DatasetType::NCLT)
        : bag_file_(std::move(bag_file)), dataset_type_(dataset_type) {
        assert(dataset_type_ != DatasetType::UNKNOWN);
    }

    using MessageProcessFunction = std::function<bool(const rosbag::MessageInstance &m)>;

    /// 一些方便直接使用的topics, messages
    using Scan2DHandle = std::function<bool(sensor_msgs::LaserScanPtr)>;
    using MultiScan2DHandle = std::function<bool(MultiScan2d::Ptr)>;
    using PointCloud2Handle = std::function<bool(sensor_msgs::PointCloud2::Ptr)>;
    using FullPointCloudHandle = std::function<bool(FullCloudPtr)>;
    using ImuHandle = std::function<bool(IMUPtr)>;
    using GNSSHandle = std::function<bool(GNSSPtr)>;
    using OdomHandle = std::function<bool(const Odom &)>;
    using LivoxHandle = std::function<bool(const livox_ros_driver::CustomMsg::ConstPtr &msg)>;

    // 遍历文件内容，调用回调函数
    void Go();

    /// 通用处理函数
    RosbagIO &AddHandle(const std::string &topic_name, MessageProcessFunction func) {
        process_func_.emplace(topic_name, func);
        return *this;
    }

    /// 2D激光处理
    RosbagIO &AddScan2DHandle(const std::string &topic_name, Scan2DHandle f) {
        return AddHandle(topic_name, [f](const rosbag::MessageInstance &m) -> bool {
            auto msg = m.instantiate<sensor_msgs::LaserScan>();
            if (msg == nullptr) {
                return false;
            }
            return f(msg);
        });
    }

    /// 多回波2D激光处理
    RosbagIO &AddMultiScan2DHandle(const std::string &topic_name, MultiScan2DHandle f) {
        return AddHandle(topic_name, [f](const rosbag::MessageInstance &m) -> bool {
            auto msg = m.instantiate<MultiScan2d>();
            if (msg == nullptr) {
                return false;
            }
            return f(msg);
        });
    }

    /// 根据数据集类型自动确定topic名称
    RosbagIO &AddAutoPointCloudHandle(PointCloud2Handle f) {
        if (dataset_type_ == DatasetType::WXB_3D) {
            return AddHandle(wxb_lidar_topic, [f, this](const rosbag::MessageInstance &m) -> bool {
                auto msg = m.instantiate<PacketsMsg>();
                if (msg == nullptr) {
                    return false;
                }

                FullCloudPtr cloud(new FullPointCloudType), cloud_out(new FullPointCloudType);
                vlp_parser_.ProcessScan(msg, cloud);
                sensor_msgs::PointCloud2::Ptr cloud_msg(new sensor_msgs::PointCloud2);
                pcl::toROSMsg(*cloud, *cloud_msg);
                return f(cloud_msg);
            });
        } else if (dataset_type_ == DatasetType::AVIA) {
            // AVIA 不能直接获取point cloud 2
            return *this;
        } else {
            return AddHandle(GetLidarTopicName(), [f](const rosbag::MessageInstance &m) -> bool {
                auto msg = m.instantiate<sensor_msgs::PointCloud2>();
                if (msg == nullptr) {
                    return false;
                }
                return f(msg);
            });
        }
    }

    /// 根据数据集自动处理RTK消息
    RosbagIO &AddAutoRTKHandle(GNSSHandle f) {
        if (dataset_type_ == DatasetType::NCLT) {
            return AddHandle(nclt_rtk_topic, [f, this](const rosbag::MessageInstance &m) -> bool {
                auto msg = m.instantiate<sensor_msgs::NavSatFix>();
                if (msg == nullptr) {
                    return false;
                }

                GNSSPtr gnss(new GNSS(msg));
                ConvertGps2UTMOnlyTrans(*gnss);
                if (std::isnan(gnss->lat_lon_alt_[2])) {
                    // 貌似有Nan
                    return false;
                }

                return f(gnss);
            });
        } else {
            // TODO 其他数据集的RTK转换关系
        }
    }

    /// point cloud 2 的处理
    RosbagIO &AddPointCloud2Handle(const std::string &topic_name, PointCloud2Handle f) {
        return AddHandle(topic_name, [f](const rosbag::MessageInstance &m) -> bool {
            auto msg = m.instantiate<sensor_msgs::PointCloud2>();
            if (msg == nullptr) {
                return false;
            }
            return f(msg);
        });
    }

    /// livox msg 处理
    RosbagIO &AddLivoxHandle(LivoxHandle f) {
        return AddHandle(GetLidarTopicName(), [f, this](const rosbag::MessageInstance &m) -> bool {
            auto msg = m.instantiate<livox_ros_driver::CustomMsg>();
            if (msg == nullptr) {
                LOG(INFO) << "cannot inst: " << m.getTopic();
                return false;
            }
            return f(msg);
        });
    }

    /// wxb的velodyne packets处理
    RosbagIO &AddVelodyneHandle(const std::string &topic_name, FullPointCloudHandle f) {
        return AddHandle(topic_name, [f, this](const rosbag::MessageInstance &m) -> bool {
            auto msg = m.instantiate<PacketsMsg>();
            if (msg == nullptr) {
                return false;
            }

            FullCloudPtr cloud(new FullPointCloudType), cloud_out(new FullPointCloudType);
            vlp_parser_.ProcessScan(msg, cloud);

            return f(cloud);
        });
    }

    /// IMU
    RosbagIO &AddImuHandle(ImuHandle f);

    /// 清除现有的处理函数
    void CleanProcessFunc() { process_func_.clear(); }

   private:
    /// 根据设定的数据集名称获取雷达名
    std::string GetLidarTopicName() const;

    /// 根据数据集名称确定IMU topic名称
    std::string GetIMUTopicName() const;

    std::map<std::string, MessageProcessFunction> process_func_;
    std::string bag_file_;
    DatasetType dataset_type_;

    // packets driver
    tools::VelodyneConvertor vlp_parser_;
};

}  // namespace sad

#endif  // SLAM_IN_AUTO_DRIVING_IO_UTILS_H
