#include "ros/ros.h"

#include "geometry_msgs/Twist.h"

#include <sstream>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "velocity_test");
  ros::NodeHandle n;
  ros::Publisher velocity_pub = n.advertise<geometry_msgs::Twist>("/industrial_robot/diff_drive_controller/cmd_vel", 1);
  ros::Rate loop_rate(10);

  int count = 0;
  while (ros::ok())
  {
    geometry_msgs::Twist vel;

    vel.linear.x = 0.0;
    vel.angular.z = 1.8;
    velocity_pub.publish(vel);

    ros::spinOnce();

    loop_rate.sleep();
    ++count;
  }
  return 0;
}
