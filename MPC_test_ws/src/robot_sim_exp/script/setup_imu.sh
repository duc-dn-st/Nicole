#!/bin/sh
# setup IMU provided it is at ACM0
# check with "dmesg | grep tty"
sudo chmod 666 /dev/ttyACM0
