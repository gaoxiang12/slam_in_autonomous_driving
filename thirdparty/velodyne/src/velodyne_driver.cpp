#include <functional>
#include <string>

#include <pcl_ros/point_cloud.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

#include "cloud_pool.h"
#include "cloud_wrapper.h"
#include "custom_log.h"
#include "packet_process.h"
#include "packet_reader.h"
#include "synchronized_queue.h"

using namespace driver::velodyne;

int main(int argc, char** argv) {
  ros::init(argc, argv, "velodyne_driver");

  ros::NodeHandle node = ros::NodeHandle();
  ros::NodeHandle priv_node = ros::NodeHandle("~");

  // get config from launch
  // this must called before packet receiver init
  priv_node.param<std::string>("lidar_name", Config::lidar_name, "top_center");
  priv_node.param<std::string>("input_type", Config::input_type, "socket");
  priv_node.param<std::string>("lidar_type", Config::lidar_type, "VLP16");
  priv_node.param<std::string>("pub_topic", Config::pub_topic,
                               "velodyne_pointcloud");
  priv_node.param<bool>("enable_pub_packets", Config::enable_pub_packets,
                        false);
  priv_node.param<std::string>("pub_packets_topic", Config::pub_packets_topic,
                               "velodyne_packets");
  priv_node.param<std::string>("frame_id", Config::frame_id, "velodyne");
  priv_node.param<std::string>("device_ip", Config::ip_addr, "192.168.1.201");
  priv_node.param<int>("port", Config::port, 3201);
  priv_node.param<std::string>("pcap_file", Config::pcap_file, "");
  priv_node.param<double>("speed", Config::speed, 1.0);
  priv_node.param<std::string>("packet_topic", Config::packets_topic,
                               "velodyne_packets");
  priv_node.param<bool>("enable_coordinate_transformation",
                        Config::enable_coordinate_transformation, false);
  // ---------------- calibration -------------------
  node.param<float>("perception_lidar_x_offset_" + Config::lidar_name,
                    CalibParams::x_offset, 0.0);
  node.param<float>("perception_lidar_y_offset_" + Config::lidar_name,
                    CalibParams::y_offset, 0.0);
  node.param<float>("perception_lidar_z_offset_" + Config::lidar_name,
                    CalibParams::z_offset, 0.0);
  node.param<float>("perception_lidar_roll_" + Config::lidar_name,
                    CalibParams::roll, 0.0);
  node.param<float>("perception_lidar_pitch_" + Config::lidar_name,
                    CalibParams::pitch, 0.0);
  node.param<float>("perception_lidar_yaw_" + Config::lidar_name,
                    CalibParams::yaw, 0.0);
  node.param<float>("car_left", CalibParams::car_left, 0.0);
  node.param<float>("car_right", CalibParams::car_right, 0.0);
  node.param<float>("car_front", CalibParams::car_front, 0.0);
  node.param<float>("car_back", CalibParams::car_back, 0.0);
  node.param<float>("car_top", CalibParams::car_top, 0.0);
  node.param<float>("car_bottom", CalibParams::car_bottom, 0.0);

  PacketConsumer* packet_consumer =
      new PacketConsumer(Config::enable_pub_packets, 180.0);
  packet_consumer->Start();
  PacketReceiver* packet_receiver = new PacketReceiver(packet_consumer);
  if (!packet_receiver->Start()) {
    MY_LOG_ERROR("init packet receive failed");
    ros::spinOnce();
    return -1;
  }

  CloudWrapper* pub_cloud_wrapper = new CloudWrapper();
  pub_cloud_wrapper->Init(Config::pub_topic, false);

  packet_consumer->PushCallback(
      [&pub_cloud_wrapper](
          pcl::PointCloud<driver::velodyne::PointXYZRRIAR>::Ptr& cloud) {
        // cloud->height = 16;
        // cloud->width = cloud->points.size() / cloud->height;
        cloud->header.frame_id = Config::frame_id;
        // ---------------------
        // TODO: too big cloud will crash cloud shared ptr, there is a bug in
        // packet process for calculate cloud boundary, fix bug is time
        // consuming, use this temperary strategy to prevent CRASH
        // ---------------------
        MY_LOG_INFO("publish cloud size:" << cloud->height << ","
                                          << cloud->width);
        if (cloud->width < 1850) {
          pub_cloud_wrapper
              ->Publish<pcl::PointCloud<driver::velodyne::PointXYZRRIAR>::Ptr>(
                  cloud);
        }
      });

  ros::Publisher pub_packets_bag;
  if (Config::enable_pub_packets) {
    pub_packets_bag = node.advertise<velodyne_msgs::VelodyneScan>(
        Config::pub_packets_topic, 1);
    packet_consumer->SetPacketBagCallback(
        [&pub_packets_bag](velodyne_msgs::VelodyneScan::Ptr& packets) {
          pub_packets_bag.publish(*packets);
        });
  }

  ros::spin();

  delete pub_cloud_wrapper;
  delete packet_receiver;
  delete packet_consumer;

  return 0;
}