#include "tools/velodyne32/include/velodyne_config.h"

namespace driver {
namespace velodyne {

float CalibParams::x_offset = 1.05;
float CalibParams::y_offset = 0.0;
float CalibParams::z_offset = 1.78;
float CalibParams::roll = 0.15;
float CalibParams::pitch = 4.0;
float CalibParams::yaw = 0.4;
float CalibParams::car_left = 1.0;
float CalibParams::car_right = -1.0;
float CalibParams::car_front = 2.0;
float CalibParams::car_back = -2.0;
float CalibParams::car_top = 2.0;
float CalibParams::car_bottom = -20.0;

// config defualt value
std::string Config::lidar_name = "top_center";
std::string Config::input_type = "socket";
std::string Config::lidar_type = "VLP16";
std::string Config::pub_topic = "velodyne_points_1";
bool Config::enable_pub_packets = false;
std::string Config::pub_packets_topic = "velodyne_packets_1";
std::string Config::frame_id = "velodyne";
std::string Config::ip_addr = "192.168.1.201";
int Config::port = 3201;
std::string Config::pcap_file = "";
double Config::speed = 1.00;
std::string Config::packets_topic = "velodyne_packets";
bool Config::enable_coordinate_transformation = false;

}
}
