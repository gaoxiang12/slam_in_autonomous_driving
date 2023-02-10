# Velodyne LIDAR scan packets.
Header           header         # standard ROS message header
VelodynePacket[] packets        # vector of raw packets
monitor_msgs/fault_vec      fault_vec     # error information