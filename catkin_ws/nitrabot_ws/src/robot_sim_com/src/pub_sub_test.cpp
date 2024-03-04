#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/Twist.h"

ros::Publisher *pub;

void obstacleCheck(const sensor_msgs::LaserScan::ConstPtr& scan)
{
  geometry_msgs::Twist vel;
  double front = scan->ranges[0];
  double back = scan->ranges[179];
  if (front <= 2.0)
  {
    vel.linear.x = -2.0;
  }
  else if (back <= 2.0)
  {
    vel.linear.x = 2.0;
  }
  else
  {
    vel.linear.x = 0.5;
  }
  pub->publish(vel);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "pub_sub_test");
  ros::NodeHandle nh;

  pub = new ros::Publisher(nh.advertise<geometry_msgs::Twist>("industrial_robot/diff_drive_controller/cmd_vel", 1000));
  ros::Subscriber sub = nh.subscribe("/lidar/scan", 1000, &obstacleCheck);

  ros::spin();

  delete pub;

  return 0;
}
