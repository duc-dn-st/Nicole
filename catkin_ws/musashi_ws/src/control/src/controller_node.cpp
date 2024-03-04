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
#include "../include/control/mpc_ackerman.h"
#include "../include/control/pure_pursuit.h"
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

    void odometryCallback(const nav_msgs::OdometryPtr &odom_msg)
    {
        std::lock_guard<std::mutex> lock_odom(odom_mutex_);
        double now = ros::Time::now().toSec();
        std::cout << "Call time odom:\t" << now << "sec" << std::endl;
        std::cout << odom_msg->pose.pose.position.x << std::endl;
        //std::cout << "Call Odom" << std::endl;
        // store robot pose
        odom_msg_ = odom_msg;
    }

    // this is a temporary function which will be used for simulation
    // but the real world robot will send encoder values differenetly (not necessarily -> depends on driver)
    void encoderCallback(const sensor_msgs::JointStatePtr &encoder_msg)
    {
        std::lock_guard<std::mutex> lock_encoder(encoder_mutex_);
        //std::cout << "Call Encoder" << std::endl;
        // store encoder values
        encoder_msg_ = encoder_msg;
    }

    void collisionAlgoCallback(const sdv_msgs::TrajectoryPtr &collision_traj_msg)
    {
        //std::lock_guard<std::mutex> lock_vfh(collision_mutex_);
        //std::cout << "Call VFH" << std::endl;
        // store trajectory
        //collision_msg_ = collision_traj_msg;
        vfh_vel_ = collision_traj_msg->points[0].velocity_mps;
        //std::cout << vfh_vel_ << std::endl;
    }

    void pathVfhCallback(const nav_msgs::PathPtr &vfh_path)
    {
        std::lock_guard<std::mutex> lock_vfh(collision_mutex_);
        //std::cout << "Call VFH" << std::endl;
        vfh_path_ = vfh_path;
    }

    void vfhGoal(const geometry_msgs::PoseStampedPtr &vfh_goal)
    {
        vfh_goal_ << vfh_goal->pose.position.x, vfh_goal->pose.position.y, tf::getYaw(vfh_goal->pose.orientation);
    }

    void callController(const ros::TimerEvent &event)
    {
        double now = ros::Time::now().toSec();
        //std::cout << "Call time:\t" << last_ - now << "sec" << std::endl;
        last_ = now;
        //std::cout << "Call Controller" << std::endl;
        // lock the data
        std::lock_guard<std::mutex> lock_encoder(encoder_mutex_);
        std::lock_guard<std::mutex> lock_odom(odom_mutex_);
        std::lock_guard<std::mutex> lock_vfh(collision_mutex_);


        // check if ok to execute
        //if (collision_msg_ == nullptr || encoder_msg_ == nullptr || odom_msg_ == nullptr)
        if (encoder_msg_ == nullptr || odom_msg_ == nullptr)
        {
            ROS_WARN("CONTROLLER_NODE_WARN: No messages received from odom or encoder.");
            return;
        }

        // take data
        // pose
        double yaw = tf::getYaw(odom_msg_->pose.pose.orientation);
        if (yaw < 0)
        {yaw += 2.0 * M_PI;}
        robot_pose_ << odom_msg_->pose.pose.position.x, odom_msg_->pose.pose.position.y, yaw;
        tf::StampedTransform transform;
        try {
            listener_.lookupTransform("/odom", "rear_axis_center", ros::Time(0), transform);
        } catch (tf::TransformException ex) {
            ROS_ERROR("%s",ex.what());
        }
        double foo, bar, rear_yaw;
        transform.getBasis().getRPY(foo, bar, rear_yaw);
        rear_axis_pose_ << transform.getOrigin().getX(), transform.getOrigin().getY(), rear_yaw;

        odom_mutex_.unlock();


        /*
        if (use_lidar_ && !collision_msg_->points.empty())
        {
            // collision trajectory
            //trajectoryPointToPoseAndTwist(collision_msg_->points[0], pose_col_ref_, twist_col_ref_);
            trajectoryToPoseAndTwist(*collision_msg_.get(), pose_col_ref_arr_, twist_col_ref_arr_);
            //std::cout << "Angular velocity at controller: " << twist_col_ref_(2) << std::endl;
            // pop first entry
            if (collision_msg_->points.size() > 1)
            {
                std::rotate(collision_msg_->points.begin(), collision_msg_->points.begin() + 1, collision_msg_->points.end());
                collision_msg_->points.pop_back();
            }
        }
        */

        collision_mutex_.unlock();

        // encoder data
        // units should be [m/s] for the velocities

        if (simulation_)
        {
            bl_wheel_vel_ = encoder_msg_->velocity[0] * RobotConstants::WHEEL_RADIUS;
            br_wheel_vel_ = encoder_msg_->velocity[1] * RobotConstants::WHEEL_RADIUS;
            bl_steer_ = encoder_msg_->position[2];
            br_steer_ = encoder_msg_->position[3];
            fl_wheel_vel_ = encoder_msg_->velocity[6] * RobotConstants::WHEEL_RADIUS;
            fr_wheel_vel_ = encoder_msg_->velocity[7] * RobotConstants::WHEEL_RADIUS;
            fl_steer_ = encoder_msg_->position[4];
            fr_steer_ = encoder_msg_->position[5];
        }
        else
        {
            bl_wheel_vel_ = encoder_msg_->velocity[2] * RobotConstants::WHEEL_RADIUS;
            br_wheel_vel_ = encoder_msg_->velocity[3] * RobotConstants::WHEEL_RADIUS;
            // bl_steer_ = encoder_msg_->position[4];
            // br_steer_ = encoder_msg_->position[6];
            bl_steer_ = 0;
            br_steer_ = 0;
            fl_wheel_vel_ = encoder_msg_->velocity[0] * RobotConstants::WHEEL_RADIUS;
            fr_wheel_vel_ = encoder_msg_->velocity[1] * RobotConstants::WHEEL_RADIUS;
            fl_steer_ = encoder_msg_->position[5];
            fr_steer_ = encoder_msg_->position[7];
        }

        //ROS_WARN("Changed entries of joints and changed subscriber name to raw_vel");

        //////////////// FOR NO ENCODER ///////////////
        /*
        bl_wheel_vel_ = 0.0;
        br_wheel_vel_ = 0.0;
        bl_steer_ = 0.0;
        br_steer_ = 0.0;
        fl_wheel_vel_ = 0.0;
        fr_wheel_vel_ = 0.0;
        fl_steer_ = 0.0;
        fr_steer_ = 0.0;
        */
        encoder_mutex_.unlock();
        
        sdv_msgs::TrajectoryFlags flag_traj_srv;
        if (flag_traj_client_.call(flag_traj_srv))
        {
            //std::cout << "Call service" << std::endl;
            emergency_stop_flag_ = flag_traj_srv.response.emergency_stop_flag;
            avoidance_flag_ = flag_traj_srv.response.avoidance_flag;
            //trajectoryPointToPoseAndTwist(flag_traj_srv.response.trajectory, pose_traj_ref_, twist_traj_ref_);
            trajectoryToPoseAndTwist(flag_traj_srv.response.trajectory, pose_traj_ref_arr_, twist_traj_ref_arr_);
            // TESTCODE //
            //avoidance_flag_ = true;
            //emergency_stop_flag_ = false;
        }
        else
        {
            // service call unsucessfull
            ROS_ERROR("CONTROLLER_NODE_ERROR: Call to service not succesfull");
            return;
        }

        //avoidance_flag_ = true;

        // check which flag is active
        if (emergency_stop_flag_)
        {
            update_vfh_trajectory_ = true;
            std::cout << "Stop Robot" << std::endl;
            // send stop signal to controller
            if (is_ackerman_)
            {
                std::cout << "Stop motion ackerman" << std::endl;
                if (Testing::USE_VFH_PP){pure_pursuit_controller_.stopMotion();}
                else if (Testing::USE_MPC) {mpc_controller_.stopMotion();}
            }
            else
            {
                controller_.stopMotion();
            }
            std::cout << "Robot stopped" << std::endl;
        }
        else if (avoidance_flag_)
        {
            std::cout << "Avoid Obstacle" << std::endl;
            update_counter_++;
            if (update_counter_ == 15)
            {
                update_counter_ = 0;
                update_vfh_trajectory_ = true;
            }
            /*
            pose_col_ref_ = pose_col_ref_arr_.at(0);
            twist_col_ref_ = twist_col_ref_arr_.at(0);
            // send new reference to controller
            if (is_ackerman_)
            {
                double linear_vel = std::hypot(twist_col_ref_(0), twist_col_ref_(1));
                double angular_vel = twist_col_ref_(2);
                //pure_pursuit_.control(robot_pose_, pose_col_ref_, linear_vel, angular_vel,
                //                      fl_wheel_vel_, fr_wheel_vel_, bl_wheel_vel_, br_wheel_vel_,
                //                      fl_steer_, fr_steer_, bl_steer_, br_steer_);

                input_traj_ref_arr_.clear();
                input_traj_ref_arr_.resize(ControlConstants::HORIZON);
                Eigen::Vector2d input_ref;
                double delta;
                for (unsigned int ii = 0; ii < ControlConstants::HORIZON; ++ii)
                {
                    linear_vel = std::hypot(twist_col_ref_arr_.at(ii)(0), twist_col_ref_arr_.at(ii)(1));
                    std::cout << "Linear vel:\t" << linear_vel << std::endl;
                    angular_vel = twist_col_ref_arr_.at(ii)(2);
                    std::cout << "Angular vel:\t" << angular_vel << std::endl;
                    delta = std::atan(angular_vel * RobotConstants::FRONT_TO_REAR_WHEEL / linear_vel);
                    std::cout << "Delta:\t" << delta << std::endl;
                    input_ref << linear_vel, delta;
                    input_traj_ref_arr_.at(ii) = input_ref;
                }

                std::cout << "Input:\t[" << input_traj_ref_arr_.at(0)(0) << ", " << input_traj_ref_arr_.at(0)(1) << "]" << std::endl;

                */
            if (is_ackerman_)
            {
                if (update_vfh_trajectory_)
                {
                    update_vfh_trajectory_ = false;
                    sdv_msgs::Trajectory vfh_trajectory;
                    pure_pursuit_controller_.generateVfhPurePursuitTrajectory(rear_axis_pose_, vfh_path_, vfh_trajectory);

                    // generate necessary files
                    double delta;
                    Eigen::Vector2d input_ref;
                    input_traj_ref_arr_.clear();
                    input_traj_ref_arr_.resize(3 * ControlConstants::HORIZON);
                    pose_col_ref_arr_.clear();
                    pose_col_ref_arr_.resize(3 * ControlConstants::HORIZON);
                    for (unsigned int ii = 0; ii < 3 * ControlConstants::HORIZON; ++ii)
                    {
                        delta = std::atan(vfh_trajectory.points[ii].heading_rate_radps * RobotConstants::FRONT_TO_REAR_WHEEL / vfh_trajectory.points[ii].velocity_mps);
                        input_ref << vfh_trajectory.points[ii].velocity_mps, delta;
                        input_traj_ref_arr_.at(ii) = input_ref;
                        //std::cout << "\nInput:\n" << input_ref << std::endl;
                        pose_col_ref_arr_.at(ii) << vfh_trajectory.points[ii].x, vfh_trajectory.points[ii].y, vfh_trajectory.points[ii].heading;
                    }
                }

                if (Testing::USE_MPC)
                {
                    mpc_controller_.control(rear_axis_pose_, pose_col_ref_arr_, input_traj_ref_arr_,
                                            fl_wheel_vel_, fr_wheel_vel_, bl_wheel_vel_, br_wheel_vel_,
                                            fl_steer_, fr_steer_, bl_steer_, br_steer_);
                    ROS_ERROR("CONTROLLER_NODE_ERROR: Cannot run MPC in avoidance mode. -> Turn off 'Testing::TEST_AVOIDANCE'.");
                }
                else if (Testing::USE_VFH_PP)
                {
                    pure_pursuit_controller_.control(rear_axis_pose_, vfh_goal_, vfh_path_, vfh_vel_,
                                                     fl_wheel_vel_, fr_wheel_vel_, bl_wheel_vel_, br_wheel_vel_,
                                                     fl_steer_, fr_steer_, bl_steer_, br_steer_);
                }
                else
                {
                    ROS_ERROR("CONTROLLER_NODE_ERROR: No controller selected in utilities.h");
                }

                // delete first entry
                input_traj_ref_arr_.erase(input_traj_ref_arr_.begin());
                pose_col_ref_arr_.erase(pose_col_ref_arr_.begin());
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
            update_vfh_trajectory_ = true;
            std::cout << "Follow Trajectory" << std::endl;
            pose_traj_ref_ = pose_traj_ref_arr_.at(0);
            twist_traj_ref_ = twist_traj_ref_arr_.at(0);
            // send trajectory reference to controller
            if (is_ackerman_)
            {
                double linear_vel = std::hypot(twist_traj_ref_(0), twist_traj_ref_(1));
                double angular_vel = twist_traj_ref_(2);
                //pure_pursuit_.control(robot_pose_, pose_traj_ref_, linear_vel, angular_vel,
                //                      fl_wheel_vel_, fr_wheel_vel_, bl_wheel_vel_, br_wheel_vel_,
                //                     fl_steer_, fr_steer_, bl_steer_, br_steer_);
                std::cout << "(v, omega) = (" << linear_vel << ", " << angular_vel << ")" << std::endl;
                /*
                input_traj_ref_arr_.clear();
                input_traj_ref_arr_.resize(ControlConstants::HORIZON);
                Eigen::Vector2d input_ref;
                double delta;
                for (unsigned int ii = 0; ii < ControlConstants::HORIZON; ++ii)
                {
                    linear_vel = std::hypot(twist_traj_ref_arr_.at(ii)(0), twist_traj_ref_arr_.at(ii)(1)) + MathConstants::EPS;
                    //std::cout << "Linear vel:\t" << linear_vel << std::endl;
                    angular_vel = twist_traj_ref_arr_.at(ii)(2);
                    //std::cout << "Angular vel:\t" << angular_vel << std::endl;
                    delta = std::atan(angular_vel * RobotConstants::FRONT_TO_REAR_WHEEL / linear_vel);
                    //std::cout << "Delta:\t" << delta << std::endl;
                    input_ref << linear_vel, delta;
                    input_traj_ref_arr_.at(ii) = input_ref;
                }
                */
                //std::cout << "Input:\t[" << input_traj_ref_arr_.at(0)(0) << ", " << input_traj_ref_arr_.at(0)(1) << "]" << std::endl;
                if (Testing::USE_MPC)
                {
                    mpc_controller_.control(rear_axis_pose_, pose_traj_ref_arr_, input_traj_ref_arr_,
                                            fl_wheel_vel_, fr_wheel_vel_, bl_wheel_vel_, br_wheel_vel_,
                                            fl_steer_, fr_steer_, bl_steer_, br_steer_);
                }
                else if (Testing::USE_OPEN_LOOP)
                {
                    open_loop_.control(rear_axis_pose_, pose_traj_ref_, linear_vel, angular_vel,
                                       fl_wheel_vel_, fr_wheel_vel_, bl_wheel_vel_, br_wheel_vel_,
                                       fl_steer_, fr_steer_, bl_steer_, br_steer_);
                }
                else
                {
                    ROS_ERROR("CONTROLLER_NODE_ERROR: No controller selected in utilities.h");
                }

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
            mpc_controller_.shutdownDAC();
            pure_pursuit_controller_.shutdownDAC();
        }
        else
        {
            controller_.shutdownDAC();
        }
    }

private:
    ros::NodeHandle nh_, private_nh_;
    // subscriber to robot localization and encoder data
    ros::Subscriber odom_sub_, encoder_sub_, collision_sub_, vfh_goal_sub_, vfh_traj_sub_;

    ros::Publisher vfh_path_pub_;
    ros::Publisher pp_line_connection_pub_;
    // vector of robot pose (x, y, theta)
    // messages
    sdv_msgs::TrajectoryPtr collision_msg_;
    sdv_msgs::TrajectoryPtr optimal_traj_msg_;
    nav_msgs::OdometryPtr odom_msg_;
    sensor_msgs::JointStatePtr encoder_msg_;
    nav_msgs::Path vfh_pp_path_msg_;
    nav_msgs::Path pp_line_connection_msg_;
    nav_msgs::PathPtr vfh_path_;

    double vfh_distance_;

    bool stitching_flag_, emergency_stop_flag_, avoidance_flag_;
    bool is_ackerman_;
    bool use_lidar_;
    bool simulation_;

    // steer is in rad and wheel_vel is in m/s
    double bl_wheel_vel_, br_wheel_vel_, bl_steer_, br_steer_, fl_wheel_vel_, fr_wheel_vel_, fl_steer_, fr_steer_;
    Eigen::Vector3d pose_col_ref_, twist_col_ref_, robot_pose_, pose_traj_ref_, twist_traj_ref_;
    std::vector<Eigen::Vector3d> pose_col_ref_arr_, twist_col_ref_arr_, pose_traj_ref_arr_, twist_traj_ref_arr_;
    std::vector<Eigen::Vector2d> input_traj_ref_arr_;

    // sampling time of the node
    double sampling_time_;
    ros::Timer periodic_timer_;
    // mutex for all variables which are affected of the multiple threads
    std::mutex odom_mutex_, encoder_mutex_, service_mutex_, collision_mutex_;
    // object of the controller
    SimpleController controller_;
    OpenLoop open_loop_;
    MpcAckerman mpc_controller_;
    PurePursuit pure_pursuit_controller_;
    // service object
    ros::ServiceClient flag_traj_client_;

    tf::TransformListener listener_;
    Eigen::Vector3d rear_axis_pose_;
    Eigen::Vector3d vfh_goal_;

    bool update_vfh_trajectory_;
    int update_counter_;

    double vfh_vel_;

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
        nh.param("is_ackerman", is_ackerman_, false);
        nh.param("use_lidar", use_lidar_, true);
        nh.param("simulation", simulation_, true);
        // subscriber
        ROS_WARN("CONTROLLER_NODE_WARN: Subscribe 'odom' and 'encoder_odom' conditions");
        //odom_sub_ = nh_.subscribe("/encoder_odom", 4, &ControllerNode::odometryCallback, this);
        odom_sub_ = nh_.subscribe("/odometry/filtered", 4, &ControllerNode::odometryCallback, this);

        if (simulation_)
        {
            encoder_sub_ = nh_.subscribe("/musashi_robot/joint_states", 4, &ControllerNode::encoderCallback, this);
        }
        else
        {
            encoder_sub_ = nh_.subscribe("/raw_vel", 4, &ControllerNode::encoderCallback, this);
        }
        // sub of VFH trajectory (needed for velocity)
        vfh_traj_sub_ = nh_.subscribe("/obstacle_avoidance/trajectory", 1, &ControllerNode::collisionAlgoCallback, this);
        // sub of vector field heading path
        collision_sub_ = nh_.subscribe("/obstacle_avoidance/heading_path", 1, &ControllerNode::pathVfhCallback, this);

        // sub to VFH goal
        vfh_goal_sub_ = nh_.subscribe("task_manager/vfh_pp_goal", 1, &ControllerNode::vfhGoal, this);
        // timer
        periodic_timer_ = private_nh_.createTimer(ros::Duration(sampling_time_), &ControllerNode::callController, this);
        // service
        flag_traj_client_ = nh_.serviceClient<sdv_msgs::TrajectoryFlags>("flags_and_trajectory");
        // preset flags
        emergency_stop_flag_ = true;
        avoidance_flag_ = false;
        stitching_flag_ = false;

        update_vfh_trajectory_ = true;
        update_counter_ = 0;
        // preset collision msg
        sdv_msgs::Trajectory foo;
        foo.points.clear();
        collision_msg_ = boost::make_shared<sdv_msgs::Trajectory>(foo);
        // set timer
        last_ = 0;

        vfh_path_pub_ = private_nh_.advertise<nav_msgs::Path>("vfh_pure_pursuit_path", 1);
        pp_line_connection_pub_ = private_nh_.advertise<nav_msgs::Path>("line_connection", 1);
        vfh_distance_ = 1.75;
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
            mpc_controller_ = MpcAckerman(nh_, private_nh_, trans_fl, trans_fr, trans_bl, trans_br, sampling_time_);
            pure_pursuit_controller_ = PurePursuit(nh_, private_nh_, trans_fl, trans_fr, trans_bl, trans_br, sampling_time_);
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
