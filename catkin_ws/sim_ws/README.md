## Simulation Environment
This runs a gazebo simulation for a differential drive robot based on the small yellow robot.

## How to run
roslaunch industrial_robot_qt industrial_robot.launch

## Topics
## Robot Position
Topic : /odom
format : nav_msgs::OdometryPtr

## Robot Velocity Command
Topic : cmd_vel
format : geometry_msgs::Twist
