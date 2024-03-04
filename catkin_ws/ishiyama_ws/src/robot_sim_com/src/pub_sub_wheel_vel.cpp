#include "ros/ros.h"
#include <industrial_robot_qt/RobotWheelVel.h>
#include <std_msgs/Float64.h>
#include <robotdata.h>

using namespace industrial_robot;

class CombineWheelVel
{
private:
    ros::NodeHandle nh_;
    ros::Publisher left_wheel_vel_;
    ros::Publisher right_wheel_vel_;

    // is in robot input values
    double left_vel_;
    double right_vel_;
public:
    CombineWheelVel(ros::NodeHandle nh)
    {
	this->nh_ = nh;
	this->left_wheel_vel_ = nh.advertise<std_msgs::Float64>("/industrial_robot/left_wheel_velocity_controller/command", 10);
	this->right_wheel_vel_ = nh.advertise<std_msgs::Float64>("/industrial_robot/right_wheel_velocity_controller/command", 10);

	this->left_vel_ = 0;
	this->right_vel_ = 0;
    }

    // publish the velocities to the gazebo model
    void sendVelocities()
    {
	std_msgs::Float64 left_wheel_vel_msg;
	std_msgs::Float64 right_wheel_vel_msg;

	left_wheel_vel_msg.data = left_vel_;
	right_wheel_vel_msg.data = right_vel_;

	left_wheel_vel_.publish(left_wheel_vel_msg);
	right_wheel_vel_.publish(right_wheel_vel_msg);
    }

    // get the velocities and change them to [m/s]
    void getWheelVel(const industrial_robot_qt::RobotWheelVel::ConstPtr &vel)
    {
	left_vel_ = vel->left_wheel * RobotParams::input_value_to_vel;
	right_vel_ = vel->right_wheel * RobotParams::input_value_to_vel;
    }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "combine_velocities");

    ros::NodeHandle nh;
    CombineWheelVel combine_wheel_vel(nh);
    ros::Subscriber subWheelSpeed = nh.subscribe("/industrial_robot/wheel_velocities", 1000, &CombineWheelVel::getWheelVel, &combine_wheel_vel);

    ros::Rate loop_rate(20);

    while(ros::ok())
    {
	ros::spinOnce();

	combine_wheel_vel.sendVelocities();

	loop_rate.sleep();
    }

    return 0;
}
