/*
 * ===============================================================================
 * obstacle_range.cpp
 * Author: Tobias Schaefle
 * Date: 27.08.20
 * Email: tobias.schaefle@gmail.com
 * -------------------------------------------------------------------------------
 * Description:
 * This library checks if an obstacle is at the robot trajectory and returns the 
 * distance if so.
 * ===============================================================================
 */

#include "../include/collision_avoidance/obstacle_range.h"

/**
 * @brief 
 * 
 * @param pose 
 * @param trajectory 
 * @param emergency_stop_flag 
 * @param avoidance_flag 
 */
void ObstacleRange::atRange(const Eigen::Vector3d &pose, const sdv_msgs::TrajectoryPtr &trajectory, bool &emergency_stop_flag, bool &avoidance_flag)
{
    if (costmap_ == nullptr)
    {
        ROS_WARN("Costmap is a nullptr.");
        return;
    }

    // calculate cell coordinates for emergency stop
    double x_pos = 1.0; //[m]
    double y_pos = 0.0; //[m]


    /*
    // map iteration for emergency stop independent of robot pose and trajectory
    for (unsigned int ii = 0; ii < costmap_->data.size(); ++ii)
    {
        if (costmap_->data[ii] > 0)
        {
            emergency_stop_flag = true;
            avoidance_flag = false;
            return;
        }
    }
    */

    double distance;

    for (unsigned int cell_x = 0; cell_x < costmap_->info.width; ++cell_x)
    {
        for (unsigned int cell_y = 0; cell_y < costmap_->info.height; ++cell_y)
        {
            unsigned int index = cell_x + cell_y * costmap_->info.width;
            if (costmap_->data[index] == 100)
            {
                distance = cell_x * costmap_->info.resolution;
                if (distance < 1.01)
                {
                    std::cout << "Emergency stop active" << std::endl;
                    emergency_stop_flag = true;
                    avoidance_flag = false;
                    return;
                }
                else
                {
                    std::cout << "Collision avoidance is active" << std::endl;
                    //ROS_WARN("Currently no flag active.");
                    emergency_stop_flag = false;
                    avoidance_flag = true;
                    return;
                }
            }
        }
    }

    // map clear reset both
    emergency_stop_flag = false;
    avoidance_flag = false;
    return;

    /*
    //double distance;

    for (const sdv_msgs::TrajectoryPoint &point : trajectory->points)
    {
        distance = std::hypot(pose.x() - point.x, pose.y() - point.y);
        //std::cout << "Point:\t[" << point.x << ", " << point.y << "]" << std::endl;
        //std::cout << "Distance between robot and trajectory point:\t" << distance << std::endl;
        if (isSafe(point, pose))
        {
            if (distance > collision_avoidance_range_)
            {
                std::cout << "Distance and collision range:\t[" << distance << ", " << collision_avoidance_range_ << "]" << std::endl;
                std::cout << "Trajectory is safe" << std::endl;
                emergency_stop_flag = false;
                avoidance_flag = false;
                return;
            }
        }
        else
        {
            if (distance < emergency_stop_range_)
            {
                std::cout << "Emergency stop active" << std::endl;
                emergency_stop_flag = true;
                avoidance_flag = false;
                return;
            }
            else if (distance < collision_avoidance_range_)
            {
                ROS_WARN("For now in collision avoidance condition emergency stop flag is active.");
                emergency_stop_flag = true;
                avoidance_flag = false;
                return;
            }
            else
            {
                // obstacle at trajectory but too far away to call emergency stop or collision avoidance
                std::cout << "No flag call needed" << std::endl;
                emergency_stop_flag = false;
                avoidance_flag = false;
                return;
            }
        }
    }
    */
}

/**
 * @brief 
 * 
 * @param position 
 * @param goal 
 * @return true 
 * @return false 
 */
bool ObstacleRange::collisionAvoidanceFinished(const Eigen::Vector2d &position, const Eigen::Vector2d &goal)
{
    // TODO check if along the linear trajectory between robot position and goal is an obstacle
    // if there is none then the colllision avoidance is finished and stithcing should continue
    return true;
}

/**
 * @brief 
 * 
 * @param point 
 * @param pose 
 * @return true 
 * @return false 
 */
bool ObstacleRange::isSafe(const sdv_msgs::TrajectoryPoint &point, const Eigen::Vector3d &pose)
{
    unsigned int idx;
    int cell_x, cell_y;

    tf::Transform orig_tf;
    tf::poseMsgToTF(costmap_->info.origin, orig_tf);

    geometry_msgs::Pose traj_pose;
    traj_pose.position.x = point.x;
    traj_pose.position.y = point.y;
    traj_pose.orientation = tf::createQuaternionMsgFromYaw(point.heading);
    tf::Pose tf_pose;
    tf::poseMsgToTF(traj_pose, tf_pose);
    tf_pose = orig_tf.inverse() * tf_pose;

    cell_x = tf_pose.getOrigin().x() / costmap_->info.resolution;
    cell_y = tf_pose.getOrigin().y() / costmap_->info.resolution;

    //cell_x = (point.x - pose.x()) / costmap_->info.resolution + costmap_->info.width / 2;
    //cell_y = (point.y - pose.y()) / costmap_->info.resolution + costmap_->info.height / 2;

    if (outOfRange(cell_x, cell_y))
    {
        // no information about trajectory point (too close or too far away hence fine)
        return true;
    }
    idx = cell_x + static_cast<int>(costmap_->info.width) * cell_y;
    if (costmap_->data[idx] == 0) {return true;}
    else
    {
        // check diagonal
        if (costmap_->data[idx] == ROBOT_COST)
        {
            // get the rotation matrix to get the exact position of each square center
            Eigen::Rotation2D<double> rot(point.heading);
            Eigen::Vector2d pos_sq;
            for (unsigned int kk = 0; kk < translations_.size(); ++kk)
            {
                pos_sq = rot.toRotationMatrix() * translations_[kk];
                cell_x = static_cast<int>(round(pos_sq(0) - pose.x() / costmap_->info.resolution) +
                                        costmap_->info.width / 2);
                cell_y = static_cast<int>(round(pos_sq(1) - pose.y() / costmap_->info.resolution) +
                                        costmap_->info.height / 2);
                if (outOfRange(cell_x, cell_y))
                {
                    return false;
                }
                else
                {
                    idx = cell_x + static_cast<int>(costmap_->info.width) * cell_y;
                    if (costmap_->data[idx] >= SQUARE_COST)
                    {
                        return false;
                    }
                }
            }
            //not in squares hence safe
            return true;
        }
        // check squares
        else if (costmap_->data[idx] >= RECTANGLE_COST)
        {
            return false;
        }
        else
        {
            ROS_ERROR("Undefined cost in costmap.");
            return false;
        }
    }
}

/**
 * @brief ObstacleRange::updateVfhGoalPose
 * @param rear_vector_pose
 * @param trajectory
 * @param goal
 */
void ObstacleRange::updateVfhGoalPose(const Eigen::Vector3d &rear_vector_pose, const Eigen::Vector3d &robot_pose, const Eigen::Vector2d &vfh_point, geometry_msgs::PoseStamped &goal)
{
    Eigen::Vector2d goal_pos(goal.pose.position.x, goal.pose.position.y);
    double angle = GeneralFunctions::relativeAngleRobotPoint(rear_vector_pose, goal_pos);
    double distance = (goal_pos - rear_vector_pose.head<2>()).norm();

    std::cout << "Goal:\n" << goal_pos << std::endl;
    std::cout << "Robot:\n" << robot_pose << std::endl;
    std::cout << "VFH Point:\n" << vfh_point << std::endl;

    // get vfh point vector and goal point vector
    Eigen::Vector2d vfh_point_vector = (robot_pose.head<2>() - vfh_point).normalized();
    Eigen::Vector2d goal_point_vector = (robot_pose.head<2>() - goal_pos).normalized();
    // get the angle
    double alpha = std::acos(goal_point_vector.dot(vfh_point_vector));
    alpha = std::min(alpha, 2 * M_PI - alpha);

    std::cout << "Alpha: " << alpha << std::endl;

    // calculate current lookahead
    double lookahead_distance = ControlConstants::VFH_MIN_GOAL_DISTANCE +
            (ControlConstants::VFH_MAX_GOAL_DISTANCE - ControlConstants::VFH_MIN_GOAL_DISTANCE) *
            (std::min(alpha, ControlConstants::VFH_MAX_ALPHA) / ControlConstants::VFH_MAX_ALPHA);

    std::cout << "Lookahead: " << lookahead_distance << std::endl;

    ////////////////// FOR TRACKING ONLY ////////////////
    if (Testing::TRACKING_ONLY)
    {
        lookahead_distance = 0.8;
    }
    /////////////////////////////////////////////////////

    // switch goal if
    if (std::abs(angle) > ControlConstants::VFH_GOAL_ANGLE || distance < lookahead_distance)
    {
        //std::cout << vfh_goal_distance_ << std::endl;
        // iterate through trajectory and set new goal
        for (unsigned int ii = traj_index_; ii < traj_msg_->points.size(); ++ii)
        {
            sdv_msgs::TrajectoryPoint point = traj_msg_->points.at(ii);
            goal_pos << point.x, point.y;
            double angle = GeneralFunctions::relativeAngleRobotPoint(rear_vector_pose, goal_pos);
            double distance = (goal_pos - rear_vector_pose.head<2>()).norm();
            if (std::abs(angle) < ControlConstants::VFH_GOAL_ANGLE && distance > lookahead_distance)
            {
                goal.pose.position.x = point.x;
                goal.pose.position.y = point.y;
                goal.pose.orientation = tf::createQuaternionMsgFromYaw(point.heading);
                goal.header.stamp = ros::Time::now();

                // update index
                traj_index_ = ii;
                break;
            }
        }
    }

}

/**
 * @brief 
 * 
 * @param cell_x 
 * @param cell_y 
 * @return true 
 * @return false 
 */
inline bool ObstacleRange::outOfRange(int cell_x, int cell_y)
{
    if (cell_x < 0 || cell_x >= static_cast<int>(costmap_->info.width) 
        || cell_y < 0 || cell_y >= static_cast<int>(costmap_->info.height))
    {
        return true;
    }
    return false;
}
