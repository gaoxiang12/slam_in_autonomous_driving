#ifndef DRIVER_VELODYNE_CLOUD_WRAPPER_H
#define DRIVER_VELODYNE_CLOUD_WRAPPER_H

#include <pcl_ros/point_cloud.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

#include "cloud_pool.h"

namespace driver {
namespace velodyne {
class CloudWrapper {
 public:
  CloudWrapper() : cloud_pool_(nullptr) {}
  ~CloudWrapper() {
    if (cloud_pool_) {
      cloud_pool_->DeletePool();
      delete cloud_pool_;
      cloud_pool_ = nullptr;
    }
  }

  void Init(std::string topic_name, bool use_ros_topic = false) {
    // shared ptr
    cloud_pool_ = new CloudPool(topic_name);
    cloud_pool_->InitPool();

    // ros publish
    use_ros_topic_ = use_ros_topic;
    if (use_ros_topic_) {
      ros::NodeHandle node;
      cloud_pub_ = node.advertise<sensor_msgs::PointCloud2>(topic_name, 1);
    }
  }

  template <typename PCloudPtr>
  void Publish(PCloudPtr& cloud_ptr) {
    // update shared ptr
    cloud_pool_->SetCloud(*cloud_ptr);
    // publish cloud by ros topic
    if (use_ros_topic_) {
      cloud_pub_.publish(*cloud_ptr);
    }
  }

 private:
  CloudPool* cloud_pool_;
  bool use_ros_topic_;
  ros::Publisher cloud_pub_;
};
}  // namespace velodyne
}  // namespace driver
#endif  // DRIVER_VELODYNE_CLOUD_WRAPPER_H