# bash start_3d_lidar.sh
# sudo chmod 666 /dev/ttyUSB2
sudo ifconfig enp111s0 192.168.0.100
source ~/catkin_ws/musashi_minibot_ws/devel/setup.bash
roslaunch nitra_robot bringup.launch
