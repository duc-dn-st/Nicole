/*
 * ===============================================================================
 * controller_node.cpp
 * Author: Tobias Schaefle
 * Date: 27.06.20
 * Email: tobias.schaefle@gmail.com
 * -------------------------------------------------------------------------------
 * Description:
 * This is the main node for the contrller of the SDV. 
 * It receives information from the task manager and also information of the robot pose 
 * and the current encoder values. It calls the controller in a fixed sampling time.
 * ===============================================================================
 */

#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <tf/tf.h>

#include <eigen3/Eigen/Core>

#include <boost/make_shared.hpp>

// #include "../include/mpc_ackerman.h"
#include "../include/pure_pursuit.h"
#include "sdv_msgs/Trajectory.h"
#include "sdv_msgs/TrajectoryPoint.h"
#include "../include/utilities/utilities.h"

#include <cmath>
#include <mutex>

class ControllerNode
{
public:
    ControllerNode(ros::NodeHandle nh, ros::NodeHandle private_nh)
    {
        
        init(nh, private_nh);
        std::cout << "Init succesfull" << std::endl;
        initController();
        std::cout << "Controller init succesfull" << std::endl;
        ros::Duration(1).sleep();
    }

    void odometryCallback(const nav_msgs::OdometryPtr &odom_msg)
    {
        std::lock_guard<std::mutex> lock_odom(odom_mutex_);
        double now = ros::Time::now().toSec();
        std::cout << "Call time odom:\t" << now << "sec" << std::endl;
        // std::cout << odom_msg->pose.pose.position.x << std::endl;
        // std::cout << "Call Odom" << std::endl;
        // store robot pose
        odom_msg_ = odom_msg;
    }

    void callController(const ros::TimerEvent &event)
    {
        double now = ros::Time::now().toSec();
        //std::cout << "Call time:\t" << last_ - now << "sec" << std::endl;
        last_ = now;
        //std::cout << "Call Controller" << std::endl;
        // lock the data
        std::lock_guard<std::mutex> lock_encoder(encoder_mutex_);


        // check if ok to execute
        //if (collision_msg_ == nullptr || encoder_msg_ == nullptr || odom_msg_ == nullptr)
        if (odom_msg_ == nullptr)
        {
            ROS_WARN("CONTROLLER_NODE_WARN: No messages received from odom or encoder.");
            return;
        }

        // take data
        // pose
        double yaw = tf::getYaw(odom_msg_->pose.pose.orientation);
        if (yaw < 0)
        {yaw += 2.0 * M_PI;}
        rear_axis_pose_ << odom_msg_->pose.pose.position.x, odom_msg_->pose.pose.position.y, yaw;
        std::cout << "yaw:" << yaw << std::endl;
        odom_mutex_.unlock();

        trajectoryCallback();

        std::cout << "Follow Trajectory" << std::endl;
        // mpc_controller_.control(robot_pose_);
        pure_pursuit_controller_.control(rear_axis_pose_, vfh_goal_, vfh_path_, vfh_vel_);


        std::cout << "Function call time:\t" << ros::Time::now().toSec() - now << "sec" << std::endl;
    }

    void trajectoryCallback()
    {   

        // lock mutex
        // std::lock_guard<std::mutex> lock_srv(service_mutex_);
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
                vfh_goal_ << goal_pos.x(), goal_pos.y(), tf::createQuaternionMsgFromYaw(point.heading);
                break;
            }
        }
    }

    /// set the trajectory
    void setTrajectory(const sdv_msgs::TrajectoryPtr &traj_msg)
    {
        traj_msg_->header = traj_msg->header;
        traj_msg_->points = std::vector<sdv_msgs::TrajectoryPoint>(traj_msg->points.begin(), traj_msg->points.end());
    }


    // update the goal position for the VFH

    // void updateVfhGoalPose(const Eigen::Vector3d &rear_vector_pose, const Eigen::Vector3d &robot_pose, const Eigen::Vector2d &vfh_point, geometry_msgs::PoseStamped &goal)
    // {
    //     Eigen::Vector2d goal_pos(goal.pose.position.x, goal.pose.position.y);
    //     double angle = GeneralFunctions::relativeAngleRobotPoint(rear_vector_pose, goal_pos);
    //     double distance = (goal_pos - rear_vector_pose.head<2>()).norm();

    //     std::cout << "Goal:\n" << goal_pos << std::endl;
    //     std::cout << "Robot:\n" << robot_pose << std::endl;
    //     std::cout << "VFH Point:\n" << vfh_point << std::endl;

    //     // get vfh point vector and goal point vector
    //     Eigen::Vector2d vfh_point_vector = (robot_pose.head<2>() - vfh_point).normalized();
    //     Eigen::Vector2d goal_point_vector = (robot_pose.head<2>() - goal_pos).normalized();
    //     // get the angle
    //     double alpha = std::acos(goal_point_vector.dot(vfh_point_vector));
    //     alpha = std::min(alpha, 2 * M_PI - alpha);

    //     std::cout << "Alpha: " << alpha << std::endl;

    //     // calculate current lookahead
    //     double lookahead_distance = ControlConstants::VFH_MIN_GOAL_DISTANCE +
    //             (ControlConstants::VFH_MAX_GOAL_DISTANCE - ControlConstants::VFH_MIN_GOAL_DISTANCE) *
    //             (std::min(alpha, ControlConstants::VFH_MAX_ALPHA) / ControlConstants::VFH_MAX_ALPHA);

    //     std::cout << "Lookahead: " << lookahead_distance << std::endl;

    //     ////////////////// FOR TRACKING ONLY ////////////////
    //     if (Testing::TRACKING_ONLY)
    //     {
    //         lookahead_distance = 0.8;
    //     }
    //     /////////////////////////////////////////////////////

    //     // switch goal if
    //     if (std::abs(angle) > ControlConstants::VFH_GOAL_ANGLE || distance < lookahead_distance)
    //     {
    //         //std::cout << vfh_goal_distance_ << std::endl;
    //         // iterate through trajectory and set new goal
    //         for (unsigned int ii = traj_index_; ii < traj_msg_->points.size(); ++ii)
    //         {
    //             sdv_msgs::TrajectoryPoint point = traj_msg_->points.at(ii);
    //             goal_pos << point.x, point.y;
    //             double angle = GeneralFunctions::relativeAngleRobotPoint(rear_vector_pose, goal_pos);
    //             double distance = (goal_pos - rear_vector_pose.head<2>()).norm();
    //             if (std::abs(angle) < ControlConstants::VFH_GOAL_ANGLE && distance > lookahead_distance)
    //             {
    //                 goal.pose.position.x = point.x;
    //                 goal.pose.position.y = point.y;
    //                 goal.pose.orientation = tf::createQuaternionMsgFromYaw(point.heading);
    //                 goal.header.stamp = ros::Time::now();

    //                 // update index
    //                 traj_index_ = ii;
    //                 break;
    //             }
    //         }
    //     }

    // }



private:
    ros::NodeHandle nh_, private_nh_;
    // subscriber to robot localization and encoder data
    ros::Subscriber odom_sub_, traj_sub_;

    ros::Publisher vfh_path_pub_;
    ros::Publisher pp_line_connection_pub_;

    // vector of robot pose (x, y, theta)
    // messages
    nav_msgs::OdometryPtr odom_msg_;
    nav_msgs::Path vfh_pp_path_msg_;
    nav_msgs::Path pp_line_connection_msg_;
    nav_msgs::PathPtr vfh_path_;

    sdv_msgs::TrajectoryPtr traj_msg;
    sdv_msgs::TrajectoryPtr traj_msg_;
    unsigned int traj_index_;

    bool simulation_;

    // steer is in rad and wheel_vel is in m/s
    Eigen::Vector3d rear_axis_pose_;
    Eigen::Vector3d vfh_goal_;

    // sampling time of the node
    double sampling_time_;
    ros::Timer periodic_timer_;
    // mutex for all variables which are affected of the multiple threads
    std::mutex odom_mutex_, encoder_mutex_, service_mutex_, collision_mutex_;
    // object of the controller
    // MpcAckerman mpc_controller_;
    PurePursuit pure_pursuit_controller_;

    // // service object
    // ros::ServiceClient flag_traj_client_;

    int update_counter_;

    double vfh_vel_ = TrajectoryParameters::PATH_VEL_LIM;

    // timer
    double last_;

    inline void trajectoryPointToPoseAndTwist(const sdv_msgs::TrajectoryPoint &traj, Eigen::Vector3d &pose, Eigen::Vector3d &twist) const
    {
        pose << traj.x, traj.y, traj.heading;
        twist << traj.x_dot, traj.y_dot, traj.heading_rate_radps;
    }

    void trajectoryToPoseAndTwist(const sdv_msgs::Trajectory &traj, std::vector<Eigen::Vector3d> &pose_arr, std::vector<Eigen::Vector3d> &twist_arr)
    {
        Eigen::Vector3d pose, twist;
        pose_arr.clear();
        twist_arr.clear();
        pose_arr.resize(ControlConstants::HORIZON);
        twist_arr.resize(ControlConstants::HORIZON);
        for (unsigned int ii = 0; ii < traj.points.size(); ++ii)
        {
            trajectoryPointToPoseAndTwist(traj.points.at(ii), pose, twist);
            pose_arr.at(ii) = pose;
            twist_arr.at(ii) = twist;
        }
    }

    void init(ros::NodeHandle nh, ros::NodeHandle private_nh)
    {

        nh_ = nh;
        private_nh_ = private_nh;

        // check parameter server
        nh.param("controller_sampling_time", sampling_time_, 0.05); //[s]


        // nh.param("simulation", simulation_, true);
        // subscriber
        ROS_WARN("CONTROLLER_NODE_WARN: Subscribe 'odom' and 'encoder_odom' conditions");
        //odom_sub_ = nh_.subscribe("/encoder_odom", 4, &ControllerNode::odometryCallback, this);
        odom_sub_ = nh_.subscribe("/odom", 4, &ControllerNode::odometryCallback, this);
        periodic_timer_ = private_nh_.createTimer(ros::Duration(sampling_time_), &ControllerNode::callController, this);
    }

    void initController()
    {
        // mpc_controller_ = MpcAckerman(nh_, private_nh_, sampling_time_);
        pure_pursuit_controller_ = PurePursuit(nh_, private_nh_, sampling_time_);
    }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "controller_node");
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");

    ControllerNode controller(nh, private_nh);

    ros::AsyncSpinner s(4);
    s.start();
    ros::waitForShutdown();

    return 0;
}
