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
#include <sensor_msgs/JointState.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <tf/tf.h>

#include <eigen3/Eigen/Core>

#include <boost/make_shared.hpp>

#include "../include/control/simple_controller.h"
#include "../include/control/open_loop.h"
#include "../include/sdv_msgs/Trajectory.h"
#include "../include/sdv_msgs/TrajectoryPoint.h"
#include "utilities/utilities.h"
#include "../include/sdv_msgs/TrajectoryFlags.h"

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

    void callController(const ros::TimerEvent &event)
    {
        double now = ros::Time::now().toSec();
        std::cout << "Call time:\t" << last_ - now << "sec" << std::endl;
        last_ = now;
        //std::cout << "Call Controller" << std::endl;

        // encoder data
        // units should be [m/s] for the velocities
        bl_wheel_vel_ = 0;
        br_wheel_vel_ = 0;
        bl_steer_ = 0;
        br_steer_ = 0;
        fl_wheel_vel_ = 0;
        fr_wheel_vel_ = 0;
        fl_steer_ = 0;
        fr_steer_ = 0;
        
        sdv_msgs::TrajectoryFlags flag_traj_srv;
        if (flag_traj_client_.call(flag_traj_srv))
        {
            emergency_stop_flag_ = flag_traj_srv.response.emergency_stop_flag;
            avoidance_flag_ = flag_traj_srv.response.avoidance_flag;
            //trajectoryPointToPoseAndTwist(flag_traj_srv.response.trajectory_point, pose_traj_ref_, twist_traj_ref_);
        }
        else
        {
            // service call unsucessfull
            ROS_ERROR("Call to service not succesfull");
            return;
        }

        // check which flag is active
        if (emergency_stop_flag_)
        {
            std::cout << "Stop Robot" << std::endl;
            // send stop signal to controller
            if (is_ackerman_)
            {
                open_loop_.stopMotion();
            }
            {
                controller_.stopMotion();
            }
        }
        else if (avoidance_flag_)
        {
            std::cout << "Avoid Obstacle" << std::endl;
            // send new reference to controller
            if (is_ackerman_)
            {
                double linear_vel = std::hypot(twist_col_ref_(0), twist_col_ref_(1));
                double angular_vel = twist_col_ref_(2);
                open_loop_.control(robot_pose_, pose_col_ref_, linear_vel, angular_vel,
                                      fl_wheel_vel_, fr_wheel_vel_, bl_wheel_vel_, br_wheel_vel_,
                                      fl_steer_, fr_steer_, bl_steer_, br_steer_);
            }
            else
            {
                controller_.control(robot_pose_, pose_col_ref_, twist_col_ref_, 
                                    fl_wheel_vel_, fr_wheel_vel_, bl_wheel_vel_, br_wheel_vel_,
                                    fl_steer_, fr_steer_, bl_steer_, br_steer_);
            }
        }
        else
        {
            std::cout << "Follow Trajectory" << std::endl;
            // send trajectory reference to controller
            if (is_ackerman_)
            {
                double linear_vel = std::hypot(twist_traj_ref_(0), twist_traj_ref_(1));
                double angular_vel = twist_traj_ref_(2);
                open_loop_.control(robot_pose_, pose_traj_ref_, linear_vel, angular_vel,
                                      fl_wheel_vel_, fr_wheel_vel_, bl_wheel_vel_, br_wheel_vel_,
                                      fl_steer_, fr_steer_, bl_steer_, br_steer_);
            }
            else
            {
                controller_.control(robot_pose_, pose_traj_ref_, twist_traj_ref_, 
                                    fl_wheel_vel_, fr_wheel_vel_, bl_wheel_vel_, br_wheel_vel_,
                                    fl_steer_, fr_steer_, bl_steer_, br_steer_);
            }
        }
        std::cout << "Function call time:\t" << ros::Time::now().toSec() - now << "sec" << std::endl;
    }

    void shutoffDevices()
    {
        if (is_ackerman_)
        {
            open_loop_.shutdownDAC();
        }
        else
        {
            controller_.shutdownDAC();
        }
    }

private:
    ros::NodeHandle nh_, private_nh_;
    // vector of robot pose (x, y, theta)
    // messages
    sdv_msgs::TrajectoryPtr optimal_traj_msg_;

    bool stitching_flag_, emergency_stop_flag_, avoidance_flag_;
    bool is_ackerman_;

    double bl_wheel_vel_, br_wheel_vel_, bl_steer_, br_steer_, fl_wheel_vel_, fr_wheel_vel_, fl_steer_, fr_steer_;
    Eigen::Vector3d pose_col_ref_, twist_col_ref_, robot_pose_, pose_traj_ref_, twist_traj_ref_;

    // sampling time of the node
    double sampling_time_;
    ros::Timer periodic_timer_;
    // mutex for all variables which are affected of the multiple threads
    std::mutex service_mutex_;
    // object of the controller
    SimpleController controller_;
    OpenLoop open_loop_;
    // service object
    ros::ServiceClient flag_traj_client_;

    // timer
    double last_;

    inline void trajectoryPointToPoseAndTwist(const sdv_msgs::TrajectoryPoint &traj, Eigen::Vector3d &pose, Eigen::Vector3d &twist) const
    {
        pose << traj.x, traj.y, traj.heading;
        twist << traj.x_dot, traj.y_dot, traj.heading_rate_radps;
    }

    void init(ros::NodeHandle nh, ros::NodeHandle private_nh)
    {
        nh_ = nh;
        private_nh_ = private_nh;
        // check parameter server
        nh.param("controller_sampling_time", sampling_time_, 0.05); //[s]
        nh.param("is_ackerman", is_ackerman_, false);
        // timer
        periodic_timer_ = private_nh_.createTimer(ros::Duration(sampling_time_), &ControllerNode::callController, this);
        // service
        flag_traj_client_ = nh_.serviceClient<sdv_msgs::TrajectoryFlags>("flags_and_trajectory");
        // preset flags
        emergency_stop_flag_ = true;
        avoidance_flag_ = false;
        stitching_flag_ = false;
        // set timer
        last_ = 0;
    }

    void initController()
    {
        tf::TransformListener listener;
        tf::StampedTransform trans_fl, trans_fr, trans_bl, trans_br;
        bool fail = true;
        while (fail)
        {
            try
            {
                ros::Duration(1.0).sleep();
                listener.lookupTransform("/base_link", "/back_left_wheel", ros::Time(0), trans_bl);
                listener.lookupTransform("/base_link", "/back_right_wheel", ros::Time(0), trans_br);
                listener.lookupTransform("/base_link", "/front_left_wheel", ros::Time(0), trans_fl);
                listener.lookupTransform("/base_link", "/front_right_wheel", ros::Time(0), trans_fr);
                fail = false;
            }
            catch (tf::TransformException &ex)
            {
                ROS_ERROR("%s", ex.what());
                ros::Duration(1.0).sleep();
            }
        }
        if (is_ackerman_)
        {
            open_loop_ = OpenLoop(nh_, private_nh_, trans_fl, trans_fr, trans_bl, trans_br, sampling_time_);
        }
        else
        {
            controller_ = SimpleController(nh_, private_nh_, trans_fl, trans_fr, trans_bl, trans_br, sampling_time_);
        }
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

    controller.shutoffDevices();

    return 0;
}
