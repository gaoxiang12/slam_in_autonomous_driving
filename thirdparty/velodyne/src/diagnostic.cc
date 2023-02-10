#include "diagnostic_constant.h"

namespace driver {
namespace velodyne {
std::unordered_map<int, ros::Time> DiagnosticQueue::error_map;
boost::mutex DiagnosticQueue::mutex;
}  // namespace velodyne
}  // namespace driver
