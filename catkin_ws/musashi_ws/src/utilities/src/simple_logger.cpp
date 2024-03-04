/*
 * ===============================================================================
 * simple_logger.cpp
 * Author: Schaefle Tobias
 * Date: 19.04.2021
 * -------------------------------------------------------------------------------
 * Description:
 * This file is a simple data logger that subscribes to the wanted
 * topic and stores the data in a .csv file
 * ===============================================================================
 */

#include <fstream>
#include <ctime>
#include <iostream>
#include <sys/stat.h>
#include <sys/types.h>
#include <bits/stdc++.h>

#include "sdv_msgs/Trajectory.h"
#include "sdv_msgs/TrajectoryPoint.h"
#include "sdv_msgs/ControlReference.h"
#include "utilities/utilities.h"

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <tf/tf.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <sensor_msgs/JointState.h>

#include <boost/shared_ptr.hpp>

class SimpleLogger
{
public:
    SimpleLogger(ros::NodeHandle nh)
    {
        nh_ = nh;

        nh_.param("simulation", simulation_, false);

        sub_odom_ = nh_.subscribe("odometry/filtered", 1, &SimpleLogger::odomCallback, this);
        sub_encoder_odom_ = nh_.subscribe("odometry/filtered", 1, &SimpleLogger::encoderOdomCallback, this);
        sub_ground_truth_ = nh_.subscribe("odom", 1, &SimpleLogger::groundTruthCallback, this);
        sub_encoder_ = nh_.subscribe("/raw_vel", 1, &SimpleLogger::encoderCallback, this);
        sub_tracking_control_ = nh_.subscribe("/reference", 1, &SimpleLogger::trackingControlCallback, this);


        std::time_t now = time(0);
        std::tm *gmtm = gmtime(&now);
        std::string year = std::to_string(gmtm->tm_year + 1900);
        std::string month = std::to_string(gmtm->tm_mon + 1);
        std::string day = std::to_string(gmtm->tm_mday);
        // GMT+9 = Japan Time
        std::string hour = std::to_string((gmtm->tm_hour + 9) % 24);
        std::string min = std::to_string(gmtm->tm_min + 1);

        std::string store_file = "/home/musashi_03/musashi_ws/logs/";
        std::string date = day + "_" + month + "_" + year + "_" + hour + "h" + min + "min";

        // create folder
        std::string folder_path = store_file + date;
        int success = mkdir(folder_path.c_str(), 0777);
        if (success == -1)
        {
            ROS_ERROR("No folder was created");
        }

        store_file += date + "/";

        // path to store the files
        std::string odom_path = store_file + "odom_logs_" + date + ".csv";
        std::string encoder_odom_path = store_file + "encoder_odom_logs_" + date + ".csv";
        std::string trajectory_path = store_file + "trajectory_logs_" + date + ".csv";
        std::string ground_truth_path = store_file + "ground_truth_logs_" + date + ".csv";
        std::string rear_axis_path = store_file + "rear_axis_logs_" + date + ".csv";
        std::string encoder_path = store_file + "encoder_logs_" + date + ".csv";
        std::string tracking_control_path = store_file + "tracking_control_logs_" + date + ".csv";

        std::string odom_header = "time_stamp [], x_pos [m], y_pos [m], heading [rad], x_vel [m/s], y_vel [m/s], ang_vel [rad/s]";
        std::string encoder_odom_header = "time_stamp [], x_pos [m], y_pos [m], heading [rad], x_vel [m/s], y_vel [m/s], ang_vel [rad/s]";
        std::string ground_truth_header = "time_stamp [], x_pos [m], y_pos [m], heading [rad], x_vel [m/s], y_vel [m/s], ang_vel [rad/s]";
        std::string trajectory_header = "time_stamp [], traj_point [], x_pos [m], y_pos [m], heading [rad], lin_vel [m/s], ang_vel [rad/s]";
        std::string rear_axis_header = "x_pos [m], y_pos [m], heading [rad]";
        std::string encoder_header = "vel_fl [m/s], vel_fr [m/s], vel_bl [m/s], vel_br [m/s], steer_fl [rad], steer_fr [rad], steer_bl [rad], steer_br [rad]";
        std::string tracking_control_header = "vel_ref_fl [m/s], vel_ref_fr [m/s], vel_ref_bl [m/s], vel_ref_br [m/s], steer_ref_fl [rad], steer_ref_fr [rad], steer_ref_bl [rad], steer_ref_br [rad], stop [?]";

        // set up and open export files
        export_odom_.open(odom_path, std::ios::out | std::ios::trunc);
        export_odom_ << odom_header << std::endl;

        export_encoder_odom_.open(encoder_odom_path, std::ios::out | std::ios::trunc);
        export_encoder_odom_ << encoder_odom_header << std::endl;
        
	export_trajectory_.open(trajectory_path, std::ios::out | std::ios::trunc);
        export_trajectory_ << trajectory_header << std::endl;

        export_ground_truth_.open(ground_truth_path, std::ios::out | std::ios::trunc);
        export_ground_truth_ << ground_truth_header << std::endl;

        export_rear_axis_.open(rear_axis_path, std::ios::out | std::ios::trunc);
        export_rear_axis_ << rear_axis_header << std::endl;

        export_encoder_.open(encoder_path, std::ios::out | std::ios::trunc);
        export_encoder_ << encoder_header << std::endl;

        export_tracking_control_.open(tracking_control_path, std::ios::out | std::ios::trunc);
        export_tracking_control_ << tracking_control_header << std::endl;

        // call trajectory
        boost::shared_ptr<sdv_msgs::Trajectory const> trajectory_msg;
        trajectory_msg = ros::topic::waitForMessage<sdv_msgs::Trajectory>("/offline_trajectory_node/trajectory", nh_, ros::Duration(30.0));
        if (trajectory_msg == nullptr)
        {
            ROS_ERROR("SIMPLE_LOGGER_ERROR: No trajectory message received.");
        }
        else
        {
            for (auto const &point : trajectory_msg->points)
            {
                export_trajectory_ << trajectory_msg->header.stamp << "," << point.trajectory_point << "," << point.x << "," << point.y << ","
                                   << point.heading << "," << point.velocity_mps << "," << point.heading_rate_radps << std::endl;
            }
        }
    }

    void odomCallback(const nav_msgs::Odometry::ConstPtr &odom)
    {
        double yaw = tf::getYaw(odom->pose.pose.orientation);

        export_odom_ << odom->header.stamp << "," << odom->pose.pose.position.x << "," << odom->pose.pose.position.y << "," << yaw << ","
                     << odom->twist.twist.linear.x  << "," << odom->twist.twist.linear.y  << "," << odom->twist.twist.angular.z << std::endl;
    }
    
    void encoderOdomCallback(const nav_msgs::Odometry::ConstPtr &odom)
    {
        double yaw = tf::getYaw(odom->pose.pose.orientation);

        export_encoder_odom_ << odom->header.stamp << "," << odom->pose.pose.position.x << "," << odom->pose.pose.position.y << "," << yaw << ","
                     << odom->twist.twist.linear.x  << "," << odom->twist.twist.linear.y  << "," << odom->twist.twist.angular.z << std::endl;
    }

    void groundTruthCallback(const nav_msgs::Odometry::ConstPtr &ground_truth)
    {
        double yaw = tf::getYaw(ground_truth->pose.pose.orientation);

        export_ground_truth_ << ground_truth->header.stamp << "," << ground_truth->pose.pose.position.x << "," << ground_truth->pose.pose.position.y << "," << yaw << ","
                             << ground_truth->twist.twist.linear.x  << "," << ground_truth->twist.twist.linear.y  << "," << ground_truth->twist.twist.angular.z << std::endl;
    }

    void encoderCallback(const sensor_msgs::JointStatePtr &encoder_msg)
    {
        float bl_wheel_vel, br_wheel_vel, bl_steer, br_steer,
                fl_wheel_vel, fr_wheel_vel, fl_steer, fr_steer;
        if (simulation_)
        {
            bl_wheel_vel = encoder_msg->velocity[0] * RobotConstants::WHEEL_RADIUS;
            br_wheel_vel = encoder_msg->velocity[1] * RobotConstants::WHEEL_RADIUS;
            bl_steer = GeneralFunctions::wrapTo2Pif(encoder_msg->position[2]);
            br_steer = GeneralFunctions::wrapTo2Pif(encoder_msg->position[3]);
            fl_wheel_vel = encoder_msg->velocity[6] * RobotConstants::WHEEL_RADIUS;
            fr_wheel_vel = encoder_msg->velocity[7] * RobotConstants::WHEEL_RADIUS;
            fl_steer = GeneralFunctions::wrapTo2Pif(encoder_msg->position[4]);
            fr_steer = GeneralFunctions::wrapTo2Pif(encoder_msg->position[5]);
        }
        else
        {
            bl_wheel_vel = encoder_msg->velocity[2] * RobotConstants::WHEEL_RADIUS;
            br_wheel_vel = encoder_msg->velocity[3] * RobotConstants::WHEEL_RADIUS;
            bl_steer = GeneralFunctions::wrapTo2Pif(encoder_msg->position[4]);
            br_steer = GeneralFunctions::wrapTo2Pif(encoder_msg->position[6]);
            fl_wheel_vel = encoder_msg->velocity[0] * RobotConstants::WHEEL_RADIUS;
            fr_wheel_vel = encoder_msg->velocity[1] * RobotConstants::WHEEL_RADIUS;
            fl_steer = GeneralFunctions::wrapTo2Pif(encoder_msg->position[5]);
            fr_steer = GeneralFunctions::wrapTo2Pif(encoder_msg->position[7]);
        }

        export_encoder_ << fl_wheel_vel << "," << fr_wheel_vel << "," << bl_wheel_vel << "," << br_wheel_vel << ","
                        << fl_steer << "," << fr_steer << "," << bl_steer << "," << br_steer << std::endl;
    }

    void trackingControlCallback(const sdv_msgs::ControlReferenceConstPtr ref_msg)
    {
        export_tracking_control_ << ref_msg->velocity_ref_front_left << "," << ref_msg->velocity_ref_front_right << "," << ref_msg->velocity_ref_back_left << "," << ref_msg->velocity_ref_back_right << ","
                                 << ref_msg->steering_ref_front_left << "," << ref_msg->steering_ref_front_right << "," << ref_msg->steering_ref_back_left << "," << ref_msg->steering_ref_back_right << ","
                                 << ref_msg->stop_motors << std::endl;
    }

    void getRearAxis()
    {
        tf::StampedTransform transform;
        try {
            listener_.lookupTransform("/odom", "rear_axis_center", ros::Time(0), transform);
        } catch (tf::TransformException ex) {
            ROS_ERROR("%s",ex.what());
        }
        double foo, bar, rear_yaw;
        transform.getBasis().getRPY(foo, bar, rear_yaw);
        export_rear_axis_ << transform.getOrigin().getX() << "," << transform.getOrigin().getY() << "," << rear_yaw << std::endl;
    }

private:
    tf::TransformListener listener_;

    std::ofstream export_odom_;
    std::ofstream export_encoder_odom_;
    std::ofstream export_trajectory_;
    std::ofstream export_ground_truth_;
    std::ofstream export_rear_axis_;
    std::ofstream export_encoder_;
    std::ofstream export_tracking_control_;

    ros::NodeHandle nh_;

    ros::Subscriber sub_odom_;
    ros::Subscriber sub_encoder_odom_;
    ros::Subscriber sub_ground_truth_;
    ros::Subscriber sub_encoder_;
    ros::Subscriber sub_tracking_control_;

    bool simulation_;
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "logger");

    ros::NodeHandle nh;
    double sampling_time = 0.05;

    SimpleLogger logger(nh);

    ros::Rate loop_rate(1/sampling_time);

    while (ros::ok())
    {
        ros::spinOnce();

        logger.getRearAxis();

        loop_rate.sleep();
    }

    return 0;
}
