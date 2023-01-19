#!/bin/bash

roslaunch unitree_legged_real data_collection.launch
rosbag record -a --tcpnodelay