/*
 * ===============================================================================
 * robot_velocities_node.cpp
 * Author: Schaefle Tobias
 * -------------------------------------------------------------------------------
 * Description:
 * this node publishes linear and angular velocities of the robot which can be
 * used for the navigation stack planners or the rqt steering graph
 *
 * Subscribes to:
 * - /industrial_robot/wheel_velocities
 *
 * Publishes:
 * - Data type: geometry_msgs/Twist
 * - Address: cmd_vel
 * ===============================================================================
 */


// Include files from ROS libraries
#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "std_msgs/Float64.h"
#include <cmath>

// Include files of self-made libraries
#include <robotdata.h>

// Include files of other libraries

using namespace industrial_robot;

class RobotVel
{

private:

    // ROS vars
    ros::NodeHandle nh_;
    ros::Publisher left_wheel_vel_;
    ros::Publisher right_wheel_vel_;

    // Publisher vars
    std_msgs::Float64 left_wheel_vel_msg_;
    std_msgs::Float64 right_wheel_vel_msg_;

    // Subscriber vars


    // Other vars

    double prev_left_wheel_vel_;
    double prev_right_wheel_vel_;

    double linear_vel_;
    double angular_vel_;

    double sampling_time_;

    // Private methods
    // get velocitz based on max acc
    double getVelBasedOnMaxAcc(double prev_vel, double current_vel)
    {
	double acc = (current_vel - prev_vel) / sampling_time_;
	ROS_INFO("%f", sampling_time_);
	if (std::abs(acc) >= RobotParams::max_defined_ang_acc)
	{
	    current_vel = prev_vel + copysign(1.0, acc) * RobotParams::max_defined_ang_acc * sampling_time_;
	}
	return current_vel;
    }

public:

    RobotVel(ros::NodeHandle nh, double sampling_time)
    {
	this->nh_ = nh;
	this->left_wheel_vel_ = nh.advertise<std_msgs::Float64>("/industrial_robot/left_wheel_velocity_controller/command", 10);
	this->right_wheel_vel_ = nh.advertise<std_msgs::Float64>("/industrial_robot/right_wheel_velocity_controller/command", 10);

	this->left_wheel_vel_msg_.data = 0;
	this->right_wheel_vel_msg_.data = 0;

	this->prev_left_wheel_vel_ = 0.0;
	this->prev_right_wheel_vel_ = 0.0;

	this->linear_vel_ = 0.0;
	this->angular_vel_ = 0.0;

	this->sampling_time_ = sampling_time;
    }

    void getRobotVelCallback(const geometry_msgs::Twist::ConstPtr &twist_msg)
    {
	linear_vel_ = twist_msg->linear.x;
	angular_vel_ = twist_msg->angular.z;
    }

    void publishWheelVel()
    {
	double * wheel_vel;
	// in rad/s
	wheel_vel = RobotModel::robot_velocites_to_wheel_velocities(linear_vel_, angular_vel_);

	// check if acc constraint is satisfied
	left_wheel_vel_msg_.data = getVelBasedOnMaxAcc(prev_left_wheel_vel_, wheel_vel[0]);
	right_wheel_vel_msg_.data = getVelBasedOnMaxAcc(prev_right_wheel_vel_, wheel_vel[1]);

	// update prev vel
	prev_left_wheel_vel_ = left_wheel_vel_msg_.data;
	prev_right_wheel_vel_ = right_wheel_vel_msg_.data;

	left_wheel_vel_.publish(left_wheel_vel_msg_);
	right_wheel_vel_.publish(right_wheel_vel_msg_);
    }
};


int main(int argc, char **argv)
{
    double sampling_rate = 20.0;	//Hz
    double sampling_time = 1 / sampling_rate;
    ros::init(argc, argv, "robot_velocities_node");
    ros::NodeHandle nh;
    RobotVel robot_vel_node(nh, sampling_time);
    ros::Subscriber sub = nh.subscribe("cmd_vel", 10, &RobotVel::getRobotVelCallback, &robot_vel_node);

    ros::Rate loop_rate(sampling_rate);

    while (ros::ok())
    {
	ros::spinOnce();
	robot_vel_node.publishWheelVel();
	loop_rate.sleep();
    }

    ros::spin();

    // Exit
    return 0;
}
