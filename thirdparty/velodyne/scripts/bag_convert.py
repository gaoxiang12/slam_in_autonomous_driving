#!/usr/bin/env python
# convert VelodyneScan from monitor version to non-monitor version 

from __future__ import print_function

import sys
import os

import rosbag
from velodyne_msgs.msg import VelodyneScan


if __name__=='__main__':
    path = ""
    if len(sys.argv) == 2:
        path = sys.argv[1]
    files = os.listdir(path)
    for file in files:
        file_path = os.path.join(path, file)
        if not os.path.isdir(file_path):
            out_path = file_path.replace(".bag", "_out.bag")
            in_bag = rosbag.Bag(file_path)
            out_bag = rosbag.Bag(out_path,'w')
            for (topic, msg, t) in in_bag.read_messages(raw=False):
                if topic == "/velodyne_packets_1":
                    new_msg = VelodyneScan()
                    new_msg.header = msg.header 
                    new_msg.packets = msg.packets 
                    out_bag.write(topic, new_msg, t)
                else:
                    out_bag.write(topic, msg, t)

            out_bag.close()
            print(file_path, out_path)

