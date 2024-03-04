#ifndef MPC_ACKERMAN_H
#define MPC_ACKERMAN_H

#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <tf/transform_datatypes.h>

#include <Eigen/Dense>
#include <Eigen/Core>

#include <cmath>
#include <algorithm>
#include <mutex>
#include <vector>
#include <string>
#include <fstream>
#include <time.h>
#include <ctime>

#include <geometry_msgs/Twist.h>
#include "../include/control/controller_base.h"
#include <nav_msgs/Path.h>
#include <tf/LinearMath/Quaternion.h>
#include <geometry_msgs/Quaternion.h>
#include "utilities/utilities.h"
#include <OsqpEigen/OsqpEigen.h>

class MpcAckerman : public ControllerBase
{
public:
    MpcAckerman(){
  /*      sampling_time_ = 0.05;
        counter_=1;
        build_mpc_cost();
        build_inequality_constraints();

       //std::cout << this->H_mpc_ << std::endl;

        read_reference();
        initialize_ekf();

        this->pose_ << 0,0,0;
        this->input_wmr_ << 0,0;   */
    }


    // default
    MpcAckerman(ros::NodeHandle nh, ros::NodeHandle private_nh,
                    tf::StampedTransform trans_fl, tf::StampedTransform trans_fr, 
                    tf::StampedTransform trans_bl, tf::StampedTransform trans_br,
                    double sampling_time): ControllerBase(nh, private_nh, trans_fl, trans_fr, trans_bl, trans_br, sampling_time)
    {
        pred_path_pub_ = private_nh.advertise<nav_msgs::Path>("pred_path", 1);
        sampling_time_ = sampling_time;
        max_acc_ = sampling_time_* RobotConstants::MAX_WHEEL_ACC;
        max_vel_ = RobotConstants::MAX_WHEEL_VEL;
        max_steer_angle_ = MathConstants::PI / 2;
        max_steer_vel_ = sampling_time_* RobotConstants::MAX_STEERING_VEL;
        build_mpc_cost();
        //std::cout << this->H_mpc_ << std::endl;

        build_A_inequality_constraints();

        time_t now = time(0);
        strftime(filename_, sizeof(filename_), "log_MPC/%Y%m%d_%H%M.csv", localtime(&now));

        std::ofstream iniCSV;
        iniCSV.open(filename_, std::ios::out|std::ios::trunc);
       // iniCSV << "Horizon : " + std::to_string(ControlConstants::HORIZON) + ", Velocity" + std::to_string(TrajectoryParameters::PATH_VEL_LIM); 
       // iniCSV << std::endl;
        iniCSV << "pose_x [m], pose_y [m], pose_theta [rad], "
                    "v [m/s], delta [rad], "
                    "v_opt [m/s], delta_opt [rad], "
                    "x_e, y_e, theta_e, x_ref, y_ref, theta_ref, v_ref [m/s], delta_ref [rad], v_k [m/s], delta_k [rad]";
        
        for (unsigned int kk = 1; kk < ControlConstants::HORIZON; ++kk)
        {
            iniCSV << ", v_k+" + std::to_string(kk) + " [m/s], delta_k+" + std::to_string(kk) + " [rad]";
        }
        iniCSV << std::endl;
    }

    ~MpcAckerman(){ }

    // main controller call
    void control(const Eigen::Vector3d &robot_pose, const std::vector<Eigen::Vector3d> &pose_ref_traj, const std::vector<Eigen::Vector2d> &input_ref_traj,
                 const double &wheel_enc_fl, const double &wheel_enc_fr, const double &wheel_enc_bl, const double &wheel_enc_br, 
                 const double &steering_enc_fl, const double &steering_enc_fr, const double &steering_enc_bl, const double &steering_enc_br);

private:
    double sampling_time_;

    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_;
    ros::Publisher velo_pub_;
    ros::Timer periodic_timer_;

    //General parameters
    // NOTE: constant varibale TODO
    double Inv_l = 1 / (RobotConstants::FRONT_TO_REAR_WHEEL); //1/l (precalculate for better efficiency)  
    //double A_MAX = 3.0; // max acceleration in m/s^-2  <<<<<CHECK

    long NUM_STATE = 3;
    long NUM_INPUT = 2;
    long NUM_EXT_STATE = 3;

    //MPC parameters
    long HORIZON = static_cast<long>(ControlConstants::HORIZON);

    //Ineq Constraints
    long H_u_num_rows;
    long H_u_num_cols;
    double max_acc_;// = sampling_time_* RobotConstants::MAX_WHEEL_ACC;
    double max_vel_;// = RobotConstants::MAX_WHEEL_VEL;
    double max_steer_angle_;// = MathConstants::PI / 2;
    double max_steer_vel_;// = sampling_time_* RobotConstants::MAX_STEERING_VEL;

    Eigen::SparseMatrix<double> H_mpc_;
    Eigen::VectorXd f_mpc_;

    std::vector<Eigen::Triplet<double, long>> Aineq_entries_;
    Eigen::VectorXd blowineq_;
    Eigen::VectorXd bupineq_;

    //Robot Position
    Eigen::Vector3d pose_;

    //Robot Inputs
    Eigen::Vector2d input_wmr_;

    //TEMP log
    Eigen::VectorXd u_opt_;

    void build_A_inequality_constraints();
    void build_B_inequality_constraints(const std::vector<Eigen::Vector2d> &input_ref_traj);
    void build_mpc_cost();
    void calculate_opt_control(const std::vector<Eigen::Vector3d> &pose_ref_traj, const std::vector<Eigen::Vector2d> &input_ref_traj);
    Eigen::Matrix3d get_A_mpc(Eigen::Vector3d& pose_ref, Eigen::Vector2d& input_ref) const;
    Eigen::Matrix<double,3,2> get_B_mpc(Eigen::Vector3d& pose_ref, Eigen::Vector2d& input_ref) const;

    void generateCSV(const std::vector<Eigen::Vector3d> &pose_ref_traj, const std::vector<Eigen::Vector2d> &input_ref_traj);
    char filename_[30];

    //generate msg from this->input_wmr_ and publish it.
    //void publishAll();
    nav_msgs::Path pred_path_msg_;
    ros::Publisher pred_path_pub_;
};

#endif
