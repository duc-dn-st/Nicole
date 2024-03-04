/*
 * ===============================================================================
 * obstacle_range.h
 * Author: Tobias Schaefle
 * Date: 27.08.20
 * Email: tobias.schaefle@gmail.com
 * -------------------------------------------------------------------------------
 * Description:
 * This library checks if an obstacle is at the robot trajectory and returns the 
 * distance if so.
 * ===============================================================================
 */

#ifndef OBSTACLE_RANGE_H
#define OBSTACLE_RANGE_H

#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>
#include <tf/tf.h>

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Eigen>
#include <eigen3/Eigen/Geometry>

#include "utilities/utilities.h"
#include "sdv_msgs/TrajectoryPoint.h"
#include "sdv_msgs/Trajectory.h"

class ObstacleRange
{
public:
    /// Constructor
    ObstacleRange() {}
    
    ObstacleRange(ros::NodeHandle nh, double sampling_time)
    {
        init(nh, sampling_time);
    }

    void init(ros::NodeHandle nh, double sampling_time)
    {
        // get distance parameters
        //nh.param<double>("collision_avoidance_range", collision_avoidance_range_, 3.0);
        //nh.param<double>("emergency_stop_range", emergency_stop_range_, 1.5);
        collision_avoidance_range_ = 3.0;
        emergency_stop_range_ = 1.5;

        sdv_msgs::Trajectory foo;
        foo.points.clear();
        traj_msg_ = boost::make_shared<sdv_msgs::Trajectory>(foo);
        traj_index_ = 0;
        // get the size of the square and rectangle
        double square_size = RobotConstants::ROBOT_WIDTH / 2;
        double rectangle_length = RobotConstants::ROBOT_WIDTH;
        double rectangle_width = RobotConstants::ROBOT_LENGTH - 2 * square_size;
        // fill the translations for the squares
        // x position
        center_to_front_left_square_trans_[0] = rectangle_width / 2 + square_size / 2;
        center_to_front_right_square_trans_[0] = center_to_front_left_square_trans_[0];
        center_to_rear_left_square_trans_[0] = - center_to_front_left_square_trans_[0];
        center_to_rear_right_square_trans_[0] = center_to_rear_left_square_trans_[0];
        // y position
        center_to_front_left_square_trans_[1] = square_size / 2;
        center_to_rear_left_square_trans_[1] = center_to_front_left_square_trans_[1];
        center_to_front_right_square_trans_[1] = - center_to_front_left_square_trans_[1];
        center_to_rear_right_square_trans_[1] = center_to_front_right_square_trans_[1];

        translations_.push_back(center_to_front_left_square_trans_);
        translations_.push_back(center_to_front_right_square_trans_);
        translations_.push_back(center_to_rear_left_square_trans_);
        translations_.push_back(center_to_rear_right_square_trans_);

        sampling_time_ = sampling_time;
    }

    // check if the robot is at emergency stop or collision avoidance range or neither
    void atRange(const Eigen::Vector3d &pose, const sdv_msgs::TrajectoryPtr &trajectory, bool &emergency_stop_flag, bool &avoidance_flag);

    // check if VFH should be still active
    bool collisionAvoidanceFinished(const Eigen::Vector2d &position, const Eigen::Vector2d &goal);

    // update the goal position for the VFH
    void updateVfhGoalPose(const Eigen::Vector3d &rear_vector_pose, const Eigen::Vector3d &robot_pose, const Eigen::Vector2d &vfh_point, geometry_msgs::PoseStamped &goal);

    /// update the costmap
    void updateCostmap(const nav_msgs::OccupancyGrid::Ptr costmap) {costmap_ = costmap;}

    /// sdet the trajectory
    void setTrajectory(const sdv_msgs::TrajectoryPtr &traj_msg)
    {
        traj_msg_->header = traj_msg->header;
        traj_msg_->points = std::vector<sdv_msgs::TrajectoryPoint>(traj_msg->points.begin(), traj_msg->points.end());
    }

private:
    // reference to the costmap
    nav_msgs::OccupancyGrid::Ptr costmap_;
    // necessary translations in robot frame for the obstacle avoidance checkup
    Eigen::Vector2d center_to_front_left_square_trans_;
    Eigen::Vector2d center_to_front_right_square_trans_;
    Eigen::Vector2d center_to_rear_left_square_trans_;
    Eigen::Vector2d center_to_rear_right_square_trans_;

    sdv_msgs::TrajectoryPtr traj_msg_;
    unsigned int traj_index_;

    double sampling_time_;

    std::vector<Eigen::Vector2d> translations_;

    double collision_avoidance_range_;
    double emergency_stop_range_;

    const int ROBOT_COST = 25;
    const int RECTANGLE_COST = 50;
    const int SQUARE_COST = 75;

    inline bool outOfRange(int cell_x, int cell_y);

    bool isSafe(const sdv_msgs::TrajectoryPoint &point, const Eigen::Vector3d &pose);

};

#endif // OBSTACLE_RANGE_H
