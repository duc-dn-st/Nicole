#include "ros/ros.h"
#include "std_msgs/String.h"
#include "nav_msgs/Odometry.h"
#include "sensor_msgs/Imu.h"
#include "tf/transform_broadcaster.h"
#include <angles/angles.h>
#include "robotdata.h"
#include "parkingcontroller_switchingapproach.h"

class SubImuOdomPubVel
{
  double yaw, pitch, roll, x_pos, y_pos;
  ros::Publisher pub;
public:
  void get_imu_orientation(const sensor_msgs::Imu::ConstPtr& imu);
  double getYaw();
  void get_odom_pos(const nav_msgs::Odometry::ConstPtr& odom);
  double getX();
  double getY();
};

void SubImuOdomPubVel::get_imu_orientation(const sensor_msgs::Imu::ConstPtr &imu)
{
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
  //ROS_INFO("IMU:\t[%f]", yaw_deg);
  //ROS_INFO("IMU:\t[%f]", getYaw() * industrial_robot::RobotParams::rad_to_deg);
}
double SubImuOdomPubVel::getYaw(){return yaw;}

void SubImuOdomPubVel::get_odom_pos(const nav_msgs::Odometry::ConstPtr &odom)
{
  x_pos = odom->pose.pose.position.x;
  y_pos = odom->pose.pose.position.y;
  //ROS_INFO("Odom:\t{%f}", getX());
}
double SubImuOdomPubVel::getX(){return x_pos;}
double SubImuOdomPubVel::getY(){return y_pos;}

int main(int argc, char **argv)
{
  double x_add = 0.0;
  double y_add = 1.0;
  double x_ref = 0.0;
  double y_ref = 0.0;

  ros::init(argc, argv, "switching_parking_control");
  ros::NodeHandle nh;
  geometry_msgs::Twist vel;
  SubImuOdomPubVel subpub;

  ros::Publisher pub = nh.advertise<geometry_msgs::Twist>("/industrial_robot/diff_drive_controller/cmd_vel", 1000);

  //ros::Subscriber sub = nh.subscribe("/industrial_robot/diff_drive_controller/odom", 1000, get_pose);
  ros::Subscriber sub = nh.subscribe("/imu", 1000, &SubImuOdomPubVel::get_imu_orientation, &subpub);
  ros::Subscriber sub2 = nh.subscribe("/industrial_robot/diff_drive_controller/odom", 1000, &SubImuOdomPubVel::get_odom_pos, &subpub);

  controller::ParkingController parkingController(subpub.getY()+y_add, x_ref, y_ref, 0.0 * industrial_robot::RobotParams::deg_to_rad, 0.5, 0.5, 0.5, 0.5, 0.9, 0.03, 1.5);

  //double *wheel_vel = parkingController.VelocityCommands(0.0, 0.0, imulistener.getYaw());
  //double *vel_command = industrial_robot::RobotModel::wheel_velocities_to_robot_velocities(wheel_vel[1], wheel_vel[0]);
  // publish the data to another node for testing

  //vel.linear.x = vel_command[0];
  //vel.angular.z = vel_command[1];
  //pub.publish(vel);

  ros::Rate loop_rate(10);


  int count = 0;
  while (ros::ok())
  {
    /*
    nav_msgs::Odometry test;
    test.pose.pose.position.x = odomlistener.getX();
    test.pose.pose.position.y = odomlistener.getY();
    pub.publish(test);
    */
    //geometry_msgs::Twist vel;

    //double *wheel_vel = parkingController.VelocityCommands(0.0, 0.0, imulistener.getYaw());
    double *wheel_vel = parkingController.VelocityCommands(subpub.getX()+x_add, subpub.getY()+y_add, subpub.getYaw());
    double *vel_command = industrial_robot::RobotModel::wheel_velocities_to_robot_velocities(wheel_vel[1], wheel_vel[0]);

    //ROS_INFO("Linear:\t[%f]",wheel_vel[0]);
    //ROS_INFO("Angular:\t[%f]",wheel_vel[1]);

    //ROS_INFO("Linear:\t[%f]",vel_command[0]);
    //ROS_INFO("Angular:\t[%f]",vel_command[1]);

    vel.linear.x = vel_command[0];
    vel.angular.z = vel_command[1];
    pub.publish(vel);

    ros::spinOnce();

    loop_rate.sleep();
    ++count;
  }

  ros::spin();

  return 0;
}
