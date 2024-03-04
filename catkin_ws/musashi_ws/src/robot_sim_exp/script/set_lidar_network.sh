#!/bin/sh
# front velodyne
sudo ifconfig enp111s0 192.168.20.101
sudo route add 192.168.20.140 enp111s0
# back velodyne
sudo ifconfig enx00e04c680ad1 192.168.20.102
sudo route add 192.168.20.150 enx00e04c680ad1
