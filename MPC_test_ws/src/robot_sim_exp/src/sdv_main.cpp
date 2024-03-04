/*
 * ===============================================================================
 * sdv_main.cpp
 * Author: Tobias Schaefle
 * Date: 07.08.20
 * Email: tobias.schaefle@gmail.com
 * -------------------------------------------------------------------------------
 * Description:
 * This is the main node/program for the SDV. It is basically the so called "Task Manager".
 * It calls the trajectory generation algorithm if necessary, it checks for collision and
 * decides if an emergency stop or avoidance has to be called. It calculates new stitching points
 * for the trajectory generation algorithm. Furthermore, a service sends new trajectories and
 * flags to the controller node for furhter processing.
 * ===============================================================================
 */

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/OccupancyGrid.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <nav_msgs/Path.h>

#include <eigen3/Eigen/Core>

#include "sdv_msgs/Trajectory.h"
#include "sdv_msgs/TrajectoryPoint.h"
#include "../include/sdv_msgs/TrajectoryFlags.h"
#include "collision_avoidance/obstacle_range.h"

#include <mutex>

class TaskManager
{
public:
    TaskManager(ros::NodeHandle nh, ros::NodeHandle private_nh)
    {
        init(nh, private_nh);
    }

    void odometryCallback(const nav_msgs::OdometryPtr &odom_msg)
    {
        // lock mutex (shared?)
        std::lock_guard<std::mutex> lock_odom(odom_mutex_);
        double yaw = tf::getYaw(odom_msg->pose.pose.orientation);
        if (yaw < 0)
        {yaw += 2.0 * M_PI;}
        robot_pose_ << odom_msg->pose.pose.position.x, odom_msg->pose.pose.position.y, yaw;

        tf::StampedTransform transform;
        while (true)
        {
            try
            {
                listener_.lookupTransform("/odom", "/rear_axis_center", ros::Time(0), transform);
                //std::cout << "Call succesful" << std::endl;
                break;
            } catch (tf::TransformException ex)
            {
                ROS_ERROR("%s",ex.what());
                ros::Duration(1.0).sleep();
            }
        }

        double foo, bar, rear_yaw;
        transform.getBasis().getRPY(foo, bar, rear_yaw);
        rear_axis_pose_ << transform.getOrigin().getX(), transform.getOrigin().getY(), rear_yaw;
        //std::cout << "Base:\n" << robot_pose_ << "\nRear:\n" << rear_axis_pose_ << std::endl;
    }

    void slopeMapCallback(const nav_msgs::OccupancyGridPtr &slope_map_msg)
    {
        // lock mutex
        std::lock_guard<std::mutex> lock_map(slope_map_mutex_);
        slope_map_msg_ = slope_map_msg;
    }

    // directly send to emergency stop function?
    void localMapCallback(const nav_msgs::OccupancyGridPtr &map_msg)
    {
        // lock mutex
        std::lock_guard<std::mutex> lock_map(map_mutex_);
        map_msg_ = map_msg;
    }

    // FIXME this is code just for the sake of testing
    // in the future the trajecory generation algorithm will be accessed with an action server
    // function will include call to stitching then to action server
    void trajectoryCallback(const sdv_msgs::TrajectoryPtr &traj_msg)
    {
        // lock mutex
        std::lock_guard<std::mutex> lock_srv(service_mutex_);
        if (traj_msg_->points.empty())
        {
            range_.setTrajectory(traj_msg);
            traj_msg_->header = traj_msg->header;
            traj_msg_->points = std::vector<sdv_msgs::TrajectoryPoint>(traj_msg->points.begin(), traj_msg->points.end());
            Eigen::Vector2d goal_pos;
            double distance;
            for (auto const &point : traj_msg_->points)
            {
                goal_pos << point.x, point.y;
                std::cout << "Goal:\n" << goal_pos << std::endl;
                distance = (goal_pos - rear_axis_pose_.head<2>()).norm();
                std::cout << "Distance: " << distance << std::endl;
                if (distance > ControlConstants::VFH_MIN_GOAL_DISTANCE)
                {
                    vfh_pp_goal_msg_.pose.position.x = goal_pos.x();
                    vfh_pp_goal_msg_.pose.position.y = goal_pos.y();
                    vfh_pp_goal_msg_.pose.orientation = tf::createQuaternionMsgFromYaw(point.heading);
                    vfh_pp_goal_msg_.header.frame_id = "odom";
                    vfh_pp_goal_msg_.header.stamp = ros::Time::now();
                    break;
                }
            }
        }
    }

    void pathVfhCallback(const nav_msgs::Path &vfh_path)
    {
        vfh_point_ << vfh_path.poses[vfh_path.poses.size()-1].pose.position.x, vfh_path.poses[vfh_path.poses.size()-1].pose.position.y;
        std::cout << "VFHVFH:\n" << vfh_point_ << std::endl;
    }

    // service
    bool sendFlagsAndTrajectory(sdv_msgs::TrajectoryFlags::Request &req,
                                sdv_msgs::TrajectoryFlags::Response &res)
    {
        // lock mutex
        std::lock_guard<std::mutex> lock_srv(service_mutex_);
        //std::cout << "In service" << std::endl;
        // set response
        res.emergency_stop_flag = emergency_stop_flag_;
        res.avoidance_flag = avoidance_flag_;
        sdv_msgs::Trajectory traj_horizon;
        traj_horizon.header = traj_msg_->header;
        if (traj_msg_->points.size() >= ControlConstants::HORIZON)
        {
            traj_horizon.points = std::vector<sdv_msgs::TrajectoryPoint>(traj_msg_->points.begin(), traj_msg_->points.begin() + ControlConstants::HORIZON);
        }
        else
        {
            // fill trajectory with last point
            traj_horizon.points = std::vector<sdv_msgs::TrajectoryPoint>(traj_msg_->points.begin(), traj_msg_->points.end());
            unsigned int missing_points = ControlConstants::HORIZON - traj_msg_->points.size();
            for (unsigned ii = 0; ii < missing_points; ++ii)
            {
                traj_horizon.points.push_back(traj_msg_->points.back());
            }
        }
        res.trajectory = traj_horizon;
        if (traj_msg_->points.size() > 1 && !emergency_stop_flag_)
        {
            std::rotate(traj_msg_->points.begin(), traj_msg_->points.begin() + 1, traj_msg_->points.end());
            traj_msg_->points.pop_back();
        }
        else {
            // just for now
            std::cout << "last trajectoryy point -> stop motors" << std::endl;
            emergency_stop_flag_ = true;
        }
        std::cout << "Trajectory size:\t" << traj_msg_->points.size() << std::endl;
        std::cout << "Current trajectory point:\t[" << traj_msg_->points.at(0).x << ", " <<
                     traj_msg_->points.at(0).y << ", " << traj_msg_->points.at(0).heading << "]" << std::endl;
        return true;
    }

    void manager(const ros::TimerEvent &event)
    {
        // lock mutex
        std::lock_guard<std::mutex> lock_odom(odom_mutex_);
        std::lock_guard<std::mutex> lock_map(map_mutex_);
        std::lock_guard<std::mutex> lock_srv(service_mutex_);
        std::lock_guard<std::mutex> lock_slope_map(slope_map_mutex_);
        if (use_lidar_)
        {
            // update slope map in range checking class
            range_.updateCostmap(slope_map_msg_);

            // calculate newest goal point for VFH and publish it
            range_.updateVfhGoalPose(rear_axis_pose_, robot_pose_, vfh_point_, vfh_pp_goal_msg_);
            vfh_goal_pub_.publish(vfh_pp_goal_msg_);

            // check if emergency flag or collision flag or neither
            if (avoidance_flag_)
            {
                /*
                // check if finished TODO
                if (range_.collisionAvoidanceFinished(Eigen::Vector2d::Zero(), Eigen::Vector2d::Zero()))
                {
                    // set stithicng to true
                    stitching_flag_ = true;
                }
                */
            }
            else
            {
                // either emergency flag is active or not
                // call if emergency flag should be active or collision avoidance should be active or neither
                range_.atRange(robot_pose_, traj_msg_, emergency_stop_flag_, avoidance_flag_);
            }

            // if true and avoidance and emergency false call stitching etc.
            if (stitching_flag_)
            {
                // call stitching algo

                // call trajectory generation algo

                // reset stitching
                stitching_flag_ = false;
            }
        }

        // force set booleans for testing different controllers or behaviours
        if (Testing::TEST_AVOIDANCE)
        {
            avoidance_flag_ = true;
            emergency_stop_flag_ = false;
        }
        else {
            // Todo: Change atRange function (avoidance_flag boolean is obsolete)
            avoidance_flag_ = false;
        }
        // just for now
        if (traj_msg_->points.size() == 1 && avoidance_flag_ == false)
        {
            //ROS_WARN("SDV_MAIN_WARN: Robot will not stop if end of trajectory is reached.");
            emergency_stop_flag_ = true;
        }
    }

private:
    ros::NodeHandle nh_, private_nh_;

    ros::Subscriber odom_sub_, map_sub_, traj_sub_, slope_map_sub_, vfh_point_sub_;

    ros::Publisher vfh_goal_pub_;

    sdv_msgs::TrajectoryPtr traj_msg_;
    nav_msgs::Odometry odom_msg_;
    nav_msgs::OccupancyGridPtr map_msg_;
    nav_msgs::OccupancyGridPtr slope_map_msg_;
    geometry_msgs::PoseStamped vfh_pp_goal_msg_;

    Eigen::Vector3d robot_pose_, rear_axis_pose_;

    ObstacleRange range_;

    ros::Timer periodic_timer_;

    bool emergency_stop_flag_, avoidance_flag_, stitching_flag_;

    std::mutex service_mutex_, map_mutex_, odom_mutex_, slope_map_mutex_;

    double sampling_time_;

    bool use_lidar_;

    Eigen::Vector2d vfh_point_;

    tf::TransformListener listener_;

    void init(ros::NodeHandle nh, ros::NodeHandle private_nh)
    {
        nh_ = nh;
        private_nh_ = private_nh;

        // set empty trajectory
        sdv_msgs::Trajectory foo;
        foo.points.clear();
        traj_msg_ = boost::make_shared<sdv_msgs::Trajectory>(foo);
        // check parameter server
        // take sampling time from controller and double it (hence manager updates half as often)
        nh.param("controller_sampling_time", sampling_time_, 0.05); //[s]
        sampling_time_ *= 2;

        nh.param("use_lidar", use_lidar_, true);

        range_.init(nh, sampling_time_ / 2);

        vfh_goal_pub_ = private_nh_.advertise<geometry_msgs::PoseStamped>("vfh_pp_goal", 1);

        // prepare subscribers
        ROS_WARN("SDV_MAIN_WARN: Subscribe 'odom' and 'encoder_odom' conditions");
        //odom_sub_ = nh_.subscribe("/encoder_odom", 1, &TaskManager::odometryCallback, this);
        odom_sub_ = nh_.subscribe("/odometry/filtered", 1, &TaskManager::odometryCallback, this);
        map_sub_ = nh_.subscribe("/pc_to_local_map", 1, &TaskManager::localMapCallback, this);
        traj_sub_ = nh_.subscribe("/offline_trajectory_node/trajectory", 1, &TaskManager::trajectoryCallback, this);
        slope_map_sub_ = nh_.subscribe("/slope_bump_detection/slope_map", 1, &TaskManager::slopeMapCallback, this);
        vfh_point_sub_ = nh_.subscribe("/obstacle_avoidance/heading_path", 1, &TaskManager::pathVfhCallback, this);
        // timer
        periodic_timer_ = private_nh_.createTimer(ros::Duration(sampling_time_), &TaskManager::manager, this);
        // preset for testing
        avoidance_flag_ = false;
        stitching_flag_ = false;
        emergency_stop_flag_ = false;
    }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "task_manager");
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");

    TaskManager manager(nh, private_nh);

    ros::ServiceServer service = nh.advertiseService("flags_and_trajectory", &TaskManager::sendFlagsAndTrajectory, &manager);

    ros::AsyncSpinner s(6);
    s.start();
    ros::waitForShutdown();

    return 0;
}
