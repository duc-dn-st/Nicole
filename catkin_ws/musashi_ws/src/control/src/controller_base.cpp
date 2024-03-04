/*
 * ===============================================================================
 * controller_base.h
 * Author: Schaefle Tobias
 * Date: 30.06.20
 * Email: tobias.schaefle@gmail.com
 * -------------------------------------------------------------------------------
 * Description:
 * This is the parent class for every controller class.
 * ===============================================================================
 */

#include "../include/control/controller_base.h"

void ControllerBase::stopMotion()
{
    if (is_sim_)
    {
        std::cout << "Stop simulation" << std::endl;
        stopSim();
    }
    else
    {
        reference_.velocity_ref_front_left = 0.0;
        reference_.velocity_ref_front_right = 0.0;
        reference_.velocity_ref_back_left = 0.0;
        reference_.velocity_ref_back_right = 0.0;

        reference_.steering_ref_front_left = 0.0;
        reference_.steering_ref_front_right = 0.0;
        reference_.steering_ref_back_left = 0.0;
        reference_.steering_ref_back_right = 0.0;
        reference_.stop_motors = true;

        reference_pub_.publish(reference_);
    }
}

void ControllerBase::publishCommands(const double wheel_cmd_fl, const double wheel_cmd_fr, const double wheel_cmd_bl, const double wheel_cmd_br, 
                         const double steering_cmd_fl, const double steering_cmd_fr, const double steering_cmd_bl, const double steering_cmd_br) const
{
    double now = ros::Time::now().toSec();
    std::cout << "Publish at:\t" << now << "sec" << std::endl;

    std_msgs::Float64 fl_wheel_msg, fr_wheel_msg, bl_wheel_msg, br_wheel_msg, fl_steer_msg, fr_steer_msg, bl_steer_msg, br_steer_msg;

    fl_wheel_msg.data = wheel_cmd_fl;
    fr_wheel_msg.data = wheel_cmd_fr;
    bl_wheel_msg.data = wheel_cmd_bl;
    br_wheel_msg.data = wheel_cmd_br;

    fl_steer_msg.data = steering_cmd_fl;
    fr_steer_msg.data = steering_cmd_fr;
    bl_steer_msg.data = steering_cmd_bl;
    br_steer_msg.data = steering_cmd_br;
        
    fl_wheel_pub_.publish(fl_wheel_msg);
    fr_wheel_pub_.publish(fr_wheel_msg);
    bl_wheel_pub_.publish(bl_wheel_msg);
    br_wheel_pub_.publish(br_wheel_msg);
    fl_steer_pub_.publish(fl_steer_msg);
    fr_steer_pub_.publish(fr_steer_msg);
    bl_steer_pub_.publish(bl_steer_msg);
    br_steer_pub_.publish(br_steer_msg);
}

void ControllerBase::stopSim()
{
    std_msgs::Float64 stop_msg;
    stop_msg.data = 0.0;
    std::cout << "Stop robot." << std::endl;
    fl_wheel_pub_.publish(stop_msg);
    //std::cout << "FLW" << std::endl;
    fr_wheel_pub_.publish(stop_msg);
    //std::cout << "FRW" << std::endl;
    bl_wheel_pub_.publish(stop_msg);
    //std::cout << "BLW" << std::endl;
    br_wheel_pub_.publish(stop_msg);
    //std::cout << "BRW" << std::endl;
    fl_steer_pub_.publish(stop_msg);
    //std::cout << "FLS" << std::endl;
    fr_steer_pub_.publish(stop_msg);
    //std::cout << "FRS" << std::endl;
    bl_steer_pub_.publish(stop_msg);
    //std::cout << "BLS" << std::endl;
    br_steer_pub_.publish(stop_msg);
    //std::cout << "BRS" << std::endl;
}

void ControllerBase::sendCommands(const double wheel_cmd_fl, const double wheel_cmd_fr, const double wheel_cmd_bl, const double wheel_cmd_br, 
                      const double steering_cmd_fl, const double steering_cmd_fr, const double steering_cmd_bl, const double steering_cmd_br,
                      double steering_enc_fl, double steering_enc_fr, double steering_enc_bl, double steering_enc_br)
{
    steering_enc_fl = GeneralFunctions::wrapToPi(steering_enc_fl);
    steering_enc_fr = GeneralFunctions::wrapToPi(steering_enc_fr);
    steering_enc_bl = GeneralFunctions::wrapToPi(steering_enc_bl);
    steering_enc_br = GeneralFunctions::wrapToPi(steering_enc_br);
    if (is_sim_)
    {
        std::cout << "send to Gazebo" << std::endl;
        publishCommands(wheel_cmd_fl, wheel_cmd_fr, wheel_cmd_bl, wheel_cmd_br,
                        steering_cmd_fl, steering_cmd_fr, steering_cmd_bl, steering_cmd_br);
    }
    else
    {
        reference_.velocity_ref_front_left = wheel_cmd_fl * RobotConstants::WHEEL_RADIUS;
        reference_.velocity_ref_front_right = wheel_cmd_fr * RobotConstants::WHEEL_RADIUS;
        reference_.velocity_ref_back_left = wheel_cmd_bl * RobotConstants::WHEEL_RADIUS;
        reference_.velocity_ref_back_right = wheel_cmd_br * RobotConstants::WHEEL_RADIUS;

        reference_.steering_ref_front_left = steering_cmd_fl;
        reference_.steering_ref_front_right = steering_cmd_fr;
        reference_.steering_ref_back_left = steering_cmd_bl;
        reference_.steering_ref_back_right = steering_cmd_br;
        reference_.stop_motors = false;

        reference_pub_.publish(reference_);

        /*
        std::cout << "send to SDV" << std::endl;
        // calculate the steering velocity
        double steering_fl_vel, steering_fr_vel, steering_bl_vel, steering_br_vel;
        std::cout << "Steering Cmd: [fl, fr]: [" << steering_cmd_fl << ", " << steering_cmd_fr << "]" << std::endl;
        std::cout << "Steering Prev Vel: [fl, fr]: [" << prev_steering_angle_fl_ << ", " << prev_steering_angle_fr_ << "]" << std::endl;
        bool use_encoder_data = false;
        if (use_encoder_data)
        {
            // get angular velocity from angle difference
            steering_fl_vel = GeneralFunctions::angularVelocity(prev_steering_angle_fl_, steering_cmd_fl, sampling_time_);
            steering_fr_vel = GeneralFunctions::angularVelocity(prev_steering_angle_fr_, steering_cmd_fr, sampling_time_);
            steering_bl_vel = GeneralFunctions::angularVelocity(prev_steering_angle_bl_, steering_cmd_bl, sampling_time_);
            steering_br_vel = GeneralFunctions::angularVelocity(prev_steering_angle_br_, steering_cmd_br, sampling_time_);
            prev_steering_angle_fl_ = steering_cmd_fl;
            prev_steering_angle_fr_ = steering_cmd_fr;
            prev_steering_angle_bl_ = steering_cmd_bl;
            prev_steering_angle_br_ = steering_cmd_br;
        }
        else
        {
            steering_fl_vel = GeneralFunctions::angularVelocity(steering_enc_fl, steering_cmd_fl, sampling_time_);
            steering_fr_vel = GeneralFunctions::angularVelocity(steering_enc_fr, steering_cmd_fr, sampling_time_);
            steering_bl_vel = GeneralFunctions::angularVelocity(steering_enc_bl, steering_cmd_bl, sampling_time_);
            steering_br_vel = GeneralFunctions::angularVelocity(steering_enc_br, steering_cmd_br, sampling_time_);
        }

        std::cout << "Steering Vel: [fl, fr]: [" << steering_fl_vel << ", " << steering_fr_vel << "]" << std::endl;
        // consider coulomb friction ... all is in [m/s]
        double lin_vel_fl = wheel_cmd_fl * RobotConstants::WHEEL_RADIUS + GeneralFunctions::sgn<double>(wheel_cmd_fl) * RobotConstants::COULOMB_FRICTION;
        double lin_vel_fr = wheel_cmd_fr * RobotConstants::WHEEL_RADIUS + GeneralFunctions::sgn<double>(wheel_cmd_fr) * RobotConstants::COULOMB_FRICTION;
        double lin_vel_bl = wheel_cmd_bl * RobotConstants::WHEEL_RADIUS + GeneralFunctions::sgn<double>(wheel_cmd_bl) * RobotConstants::COULOMB_FRICTION;
        double lin_vel_br = wheel_cmd_br * RobotConstants::WHEEL_RADIUS + GeneralFunctions::sgn<double>(wheel_cmd_br) * RobotConstants::COULOMB_FRICTION;

        motor_->sendMotorCommands(lin_vel_fl, lin_vel_fr, lin_vel_bl, lin_vel_br,
                                  steering_fl_vel, steering_fr_vel, steering_bl_vel, steering_br_vel);
        */
    }
    if (no_encoder_)
    {
        sensor_msgs::JointState joints;
        joints.position.resize(8);
        joints.velocity.resize(8);
        joints.position[2] = steering_cmd_bl;
        joints.position[3] = steering_cmd_br;
        joints.position[4] = steering_cmd_fl;
        joints.position[5] = steering_cmd_fr;
        joints.velocity[0] = wheel_cmd_bl; // / RobotConstants::WHEEL_RADIUS;
        joints.velocity[1] = wheel_cmd_br; // / RobotConstants::WHEEL_RADIUS;
        joints.velocity[6] = wheel_cmd_fl; // / RobotConstants::WHEEL_RADIUS;
        joints.velocity[7] = wheel_cmd_fr; // / RobotConstants::WHEEL_RADIUS;

        no_encoder_pub_.publish(joints);
    }
}

/**
 * @brief 
 * 
 * @param error 
 * @return true 
 * @return false 
 */
bool ControllerBase::atGoal(const Eigen::Vector3d &error) const
{
    if (std::abs(error(0)) < POS_THRESHOLD && std::abs(error(1)) < POS_THRESHOLD && std::abs(error(2)) < HEADING_THRESHOLD)
    {
        return true;
    }
    return false;
}
