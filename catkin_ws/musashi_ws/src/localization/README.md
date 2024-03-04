# Localization

This package contains various localization methods for the SDV. The localization using the kinematics of the robot (pseudo-holonomic) with the encoder readings of the drive and steering motors are employed and a launch file is provided to run sensor fusion with EKF. At the moment the sensor fusion is done with the kineamtics lcalization and the IMU data. In the future the localization of the Lidar sensors using e.g. NDT or GICP is fed to the EKF as well.

## How to run
The sensor fusion can be called with  
*roslaunch localization start_sensor_fusion.launch*  
And the kinematic localization can be called with  
*rosrun localization kinematic*