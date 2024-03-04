#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/Imu.h"
#include <sstream>
#include "robotdata.h"
#include "tf/transform_broadcaster.h"
#include <angles/angles.h>
#include "nav_msgs/Odometry.h"

using namespace industrial_robot;

/*
void chatterCallback(const sensor_msgs::LaserScan::ConstPtr& scan)
{
  ROS_INFO("position=: [%f]",scan->ranges[0]);
}
*/

void chatterCallback(const nav_msgs::Odometry::ConstPtr& kalman)
{
  /*
  ROS_INFO("Angular Velocity=:\t [%f]",scan->angular_velocity.z);
  ROS_INFO("x Acceleration=:\t [%f]",scan->linear_acceleration.x);
  ROS_INFO("y Acceleration=:\t [%f]",scan->linear_acceleration.y);
  */
  //ROS_INFO("TEST=: \t [%f]", RobotParams::TestInput());
  //ROS_INFO("TEST=: \t [%f]", RobotParams::deg_to_rad);
  //ROS_INFO("TEST=: \t [%f]", RobotModel::wheel_velocities_to_robot_velocities(1.0, 1.0)[1]);
  //ROS_INFO("TEST=: \t [%f]", RobotModel::robot_velocites_to_wheel_velocities(1.0, 1.0)[1]);
  double yaw, pitch, roll;
  // this will initial the quaternion variables which contain x, y, z, w
  tf::Quaternion quaternion;
  // convert the quaternion msg to quaternion
  //tf::quaternionMsgToTF(imu->orientation, quaternion);
  tf::quaternionMsgToTF(kalman->pose.pose.orientation, quaternion);
  // get the quaternion message and represent it in euler roll pitch yaw
  tf::Matrix3x3(quaternion).getRPY(roll, pitch, yaw);
  // normalize angle between 0 and 2*pi
  yaw = angles::normalize_angle_positive(yaw);
  // transform to deg values
  double yaw_deg = yaw * industrial_robot::RobotParams::rad_to_deg;
  ROS_INFO("Kalman:\t[%f]", yaw_deg);
}

void chatterCallbackTwo(const sensor_msgs::Imu::ConstPtr imu)
{
  /*
  ROS_INFO("Angular Velocity=:\t [%f]",scan->angular_velocity.z);
  ROS_INFO("x Acceleration=:\t [%f]",scan->linear_acceleration.x);
  ROS_INFO("y Acceleration=:\t [%f]",scan->linear_acceleration.y);
  */
  //ROS_INFO("TEST=: \t [%f]", RobotParams::TestInput());
  //ROS_INFO("TEST=: \t [%f]", RobotParams::deg_to_rad);
  //ROS_INFO("TEST=: \t [%f]", RobotModel::wheel_velocities_to_robot_velocities(1.0, 1.0)[1]);
  //ROS_INFO("TEST=: \t [%f]", RobotModel::robot_velocites_to_wheel_velocities(1.0, 1.0)[1]);
  double yaw, pitch, roll;
  // this will initial the quaternion variables which contain x, y, z, w
  tf::Quaternion quaternion;
  // convert the quaternion msg to quaternion
  tf::quaternionMsgToTF(imu->orientation, quaternion);
  // get the quaternion message and represent it in euler roll pitch yaw
  tf::Matrix3x3(quaternion).getRPY(roll, pitch, yaw);
  // normalize angle between 0 and 2*pi
  yaw = angles::normalize_angle_positive(yaw);
  // transform to deg values
  double yaw_deg = yaw * industrial_robot::RobotParams::rad_to_deg;
  ROS_INFO("IMU:\t[%f]", yaw_deg);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "measurment_test");
  ros::NodeHandle nh;
  //ros::Subscriber sub = nh.subscribe("/lidar/scan", 10, chatterCallback);
  ros::Subscriber sub = nh.subscribe("/odometry/filtered", 10, chatterCallback);
  ros::Subscriber sub2 = nh.subscribe("/imu/data", 10, chatterCallbackTwo);

  ros::spin();

  return 0;
}
