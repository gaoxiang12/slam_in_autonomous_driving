#ifndef DRIVER_VELODYNE_DIAGNOSTIC_CONSTANT_H
#define DRIVER_VELODYNE_DIAGNOSTIC_CONSTANT_H

#include <sstream>
#include <unordered_map>

#include <ros/ros.h>

#include <std_msgs/String.h>

#include <boost/thread.hpp>

namespace driver {
namespace velodyne {

enum ErrorCode {
  OK = 0,
  PERCEPTION_LIDAR_DRIVER_ALGORITHM_0_ERROR = 10001,
  PERCEPTION_LIDAR_DRIVER_ALGORITHM_1_ERROR = 10002,
  PERCEPTION_LIDAR_DRIVER_ALGORITHM_2_ERROR = 10003,
  PERCEPTION_LIDAR_DRIVER_ALGORITHM_3_ERROR = 10004,
  PERCEPTION_LIDAR_DRIVER_ALGORITHM_4_ERROR = 10005,
  PERCEPTION_LIDAR_DRIVER_ALGORITHM_5_ERROR = 10006,
};
namespace Socket {
enum SocketErrorCode {
  OK = static_cast<int>(ErrorCode::OK),  // socket input is OK
  IP_EMPTY = 1101,                       // socket input ip is empty
  IP_WRONG = 1102,  // input ip is not correct, switch ip_str to in_addr wrong
  SOCKET_CREATE_FAILED = 1103,     // create socket failed
  SOCKET_OPTION_FAILED = 1104,     // set socket option failed
  SOCKET_BIND_FAILED = 1105,       // bind socket failed
  SOCKET_NON_BLOCK_FAILED = 1106,  // set socket to non-block failed

  SOCKET_POLL_TIMEOUT = 1107,  // get socket data time out
  SOCKET_RECV_FAILED =
      1108,  // socket recvfrom failed, return nbytes is less than 0

  SOCKET_DATA_UNMATCHED_IP =
      1109,  // socket received data ip is not match desired ip

  SOCKET_DATA_INVALIDE_DATA =
      1110,  // packet data from socket port is not valid

  SOCKET_LIDAR_TYPE_IS_NOT_MATCH =
      1111,  // received packet lidar type is not matching
};
}

namespace PCAP {
enum PCAPErrorCode {
  OK = static_cast<int>(ErrorCode::OK),  // socket input is OK
  OPEN_PCAP_FILE_FAILED = 1201,          // open pcap file failed
  CREATE_COMPILE_FAILED = 1202,          // create compile failed
  SET_FILTER_FAILED = 1203,              // set filter failed
  PCAP_NOT_INIT = 1204,                  // pcap is not init
  READ_PCAP_FAILED = 1205,               // read pcap next frame failed
  PCAP_FRAME_LEN_NOT_MATCH =
      1206,  // read pcap next frame data length is not correct
};
}  // namespace PCAP

namespace BAG {
enum BagErrorCode {
  OK = static_cast<int>(ErrorCode::OK),
  PACKETS_TOPIC_EMPTY = 1301,  // packets input topic name is empty
};
}

struct ErrorInfo {
  int fault_level;
  std::string error_msg;
};

// the map from error code to fault info
const std::unordered_map<int, int> error_fault_map = {
    {static_cast<int>(Socket::SocketErrorCode::IP_EMPTY),
     static_cast<int>(ErrorCode::PERCEPTION_LIDAR_DRIVER_ALGORITHM_0_ERROR)},
    {static_cast<int>(Socket::SocketErrorCode::IP_WRONG),
     static_cast<int>(ErrorCode::PERCEPTION_LIDAR_DRIVER_ALGORITHM_0_ERROR)},
    {static_cast<int>(Socket::SocketErrorCode::SOCKET_CREATE_FAILED),
     static_cast<int>(ErrorCode::PERCEPTION_LIDAR_DRIVER_ALGORITHM_1_ERROR)},
    {static_cast<int>(Socket::SocketErrorCode::SOCKET_OPTION_FAILED),
     static_cast<int>(ErrorCode::PERCEPTION_LIDAR_DRIVER_ALGORITHM_1_ERROR)},
    {static_cast<int>(Socket::SocketErrorCode::SOCKET_BIND_FAILED),
     static_cast<int>(ErrorCode::PERCEPTION_LIDAR_DRIVER_ALGORITHM_1_ERROR)},
    {static_cast<int>(Socket::SocketErrorCode::SOCKET_NON_BLOCK_FAILED),
     static_cast<int>(ErrorCode::PERCEPTION_LIDAR_DRIVER_ALGORITHM_1_ERROR)},
    {static_cast<int>(Socket::SocketErrorCode::SOCKET_POLL_TIMEOUT),
     static_cast<int>(ErrorCode::PERCEPTION_LIDAR_DRIVER_ALGORITHM_2_ERROR)},
    {static_cast<int>(Socket::SocketErrorCode::SOCKET_RECV_FAILED),
     static_cast<int>(ErrorCode::PERCEPTION_LIDAR_DRIVER_ALGORITHM_2_ERROR)},
    {static_cast<int>(Socket::SocketErrorCode::SOCKET_DATA_UNMATCHED_IP),
     static_cast<int>(ErrorCode::PERCEPTION_LIDAR_DRIVER_ALGORITHM_3_ERROR)},
    {static_cast<int>(Socket::SocketErrorCode::SOCKET_DATA_INVALIDE_DATA),
     static_cast<int>(ErrorCode::PERCEPTION_LIDAR_DRIVER_ALGORITHM_4_ERROR)},
    {static_cast<int>(Socket::SocketErrorCode::SOCKET_LIDAR_TYPE_IS_NOT_MATCH),
     static_cast<int>(ErrorCode::PERCEPTION_LIDAR_DRIVER_ALGORITHM_5_ERROR)}};

const std::unordered_map<int, ErrorInfo> error_info_map = {
    {static_cast<int>(ErrorCode::PERCEPTION_LIDAR_DRIVER_ALGORITHM_0_ERROR),
     {2, "socket ip wrong"}},
    {static_cast<int>(ErrorCode::PERCEPTION_LIDAR_DRIVER_ALGORITHM_1_ERROR),
     {2, "init socket port failed"}},
    {static_cast<int>(ErrorCode::PERCEPTION_LIDAR_DRIVER_ALGORITHM_2_ERROR),
     {2, "get packet data from port failed"}},
    {static_cast<int>(ErrorCode::PERCEPTION_LIDAR_DRIVER_ALGORITHM_3_ERROR),
     {1, "packet ip address is not matching setting"}},
    {static_cast<int>(ErrorCode::PERCEPTION_LIDAR_DRIVER_ALGORITHM_4_ERROR),
     {2, "received packet data is not valid"}},
    {static_cast<int>(ErrorCode::PERCEPTION_LIDAR_DRIVER_ALGORITHM_5_ERROR),
     {2, "packet data lidar type is not correct"}}};

class DiagnosticQueue {
 public:
  static void AddError(const int error_code) {
    boost::unique_lock<boost::mutex> lock(mutex);
    error_map[error_code] = ros::Time::now();
  }

  static void CollectAndClearError(std::unordered_map<int, ros::Time>& errors) {
    boost::unique_lock<boost::mutex> lock(mutex);
    errors = error_map;
    error_map.clear();
  }

 private:
  static std::unordered_map<int, ros::Time> error_map;
  static boost::mutex mutex;  // The mutex to synchronise on
};

using MonitorMsg = std_msgs::String;
using MonitorMsgArray = std_msgs::String;

class DiagnosticDog {
 public:
  DiagnosticDog(ros::NodeHandle& node, std::string pub_topic, float period) {
    // get current node name
    node_name_ = ros::this_node::getName();
    // create dog timer to check error array
    dog_timer_ = node.createTimer(ros::Duration(period),
                                  &DiagnosticDog::TimerCallback, this);
    pub_error_info_ = node.advertise<MonitorMsgArray>(pub_topic, 1);
  }

  MonitorMsg CreateFaultInfo(int error_code, double timestamp) {
    MonitorMsg result;
    if (error_info_map.count(error_code) > 0) {
      auto error_info = error_info_map.at(error_code);
      std::stringstream ss;
      ss << "[" + node_name_ + "] " + error_info.error_msg;
      result.data = ss.str();
    }

    return result;
  }

  void TimerCallback(const ros::TimerEvent& event) {
    // check diagnostic map info
    std::unordered_map<int, ros::Time> errors;
    DiagnosticQueue::CollectAndClearError(errors);
    MonitorMsgArray error_array;
    if (errors.size() > 0) {
      for (auto& error : errors) {
        if (error_fault_map.count(error.first) > 0) {
          MonitorMsg error_msg = CreateFaultInfo(
              error_fault_map.at(error.first), error.second.toSec());

          std::stringstream ss;
          ss << error_array.data << "\n" << error_msg.data;
          error_array.data = ss.str();
        }
      }

      // publish
      pub_error_info_.publish(error_array);
    }
  }

 private:
  std::string node_name_;
  ros::Timer dog_timer_;
  ros::Publisher pub_error_info_;
};

}  // namespace velodyne
}  // namespace driver
#endif  // DRIVER_VELODYNE_DIAGNOSTIC_CONSTANT_H
