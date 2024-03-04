/*
 * ===============================================================================
 * low_level_controller.h
 * Author: Tobias Schaefle
 * Date: 07.06.20
 * Email: tobias.schaefle@gmail.com
 * -------------------------------------------------------------------------------
 * Description:
 * This is a node of the low level controllers.
 * ===============================================================================
 */

#include "../include/control/low_level_controller.h"

double VelocityController::getControlCmd(const double vel_ref, const double vel)
{
    double error = vel_ref - vel;
    std::cout << "Error: " << error << std::endl;
    integral_err_ += (error + prev_err_) / 2;

    // calculate command
    // vel_ref is the feedforward part
    double command = k_p_ * error + k_i_ * integral_err_ + vel_ref;

    // update prev error
    prev_err_ = error;

    return command;
}

double SteeringController::getControlCmd(const double steering_ref, const double steering)
{
    double error = steering_ref - steering;
    integral_err_ += (error + prev_err_) / 2;

    // calculate command
    double command = k_p_ * error + k_i_ * integral_err_;

    // update prev error
    prev_err_ = error;

    return command;
}

void LowLevelController::controlReferenceCallback(const sdv_msgs::ControlReferenceConstPtr ref_msg)
{
    //std::lock_guard<std::mutex> lock(reference_mutex_);
    reference_ = *ref_msg;
    if (reference_.stop_motors)
    {
        motor_->stopMotors();
    }
}

void LowLevelController::encoderCallback(const sensor_msgs::JointStatePtr encoder_msg)
{
    //std::lock_guard<std::mutex> lock(encoder_mutex_);
    encoder_ = encoder_msg;
    //std::cout << encoder_->velocity[0] * RobotConstants::WHEEL_RADIUS << std::endl;
}

void LowLevelController::gainCallback(control::ControlGainsConfig &config, uint32_t level)
{
    /*
    velocity_control_fl_.updateGains(config.vel_fl_k_p, config.vel_fl_k_i);
    velocity_control_fr_.updateGains(config.vel_fr_k_p, config.vel_fr_k_i);
    velocity_control_bl_.updateGains(config.vel_bl_k_p, config.vel_bl_k_i);
    velocity_control_br_.updateGains(config.vel_br_k_p, config.vel_br_k_i);

    steering_control_fl_.updateGains(config.steer_fl_k_p, config.steer_fl_k_i);
    steering_control_fr_.updateGains(config.steer_fr_k_p, config.steer_fr_k_i);
    steering_control_bl_.updateGains(config.steer_bl_k_p, config.steer_bl_k_i);
    steering_control_br_.updateGains(config.steer_br_k_p, config.steer_br_k_i);
    */
    std::cout << "Dynamic Reconfigure Called" << std::endl;
}

void LowLevelController::runController(const ros::TimerEvent &event)
{
    if (encoder_ == nullptr)
    {
        return;
    }

    if (reference_.stop_motors)
    {
        motor_->stopMotors();
        return;
    }
    // store reference data
    //std::unique_lock<std::mutex> lock_ref(reference_mutex_);
    //lock_ref.lock();

    vel_ref_fl_ = reference_.velocity_ref_front_left;
    vel_ref_fr_ = reference_.velocity_ref_front_right;
    vel_ref_bl_ = reference_.velocity_ref_back_left;
    vel_ref_br_ = reference_.velocity_ref_back_right;
    steer_ref_fl_ = reference_.steering_ref_front_left;
    steer_ref_fr_ = reference_.steering_ref_front_right;
    steer_ref_bl_ = reference_.steering_ref_back_left;
    steer_ref_br_ = reference_.steering_ref_back_right;

    //lock_ref.unlock();

    // store encoder data
    //std::unique_lock<std::mutex> lock_enc(encoder_mutex_);
    //lock_enc.lock();

    vel_enc_fl_ = encoder_->velocity[0] * RobotConstants::WHEEL_RADIUS;
    vel_enc_fr_ = encoder_->velocity[1] * RobotConstants::WHEEL_RADIUS;
    vel_enc_bl_ = encoder_->velocity[2] * RobotConstants::WHEEL_RADIUS;
    vel_enc_br_ = encoder_->velocity[3] * RobotConstants::WHEEL_RADIUS;

    steer_enc_fl_ = GeneralFunctions::wrapToPif(encoder_->position[5]);
    steer_enc_fr_ = GeneralFunctions::wrapToPif(encoder_->position[7]);
    steer_enc_bl_ = GeneralFunctions::wrapToPif(encoder_->position[4]);
    steer_enc_br_ = GeneralFunctions::wrapToPif(encoder_->position[6]);

    //lock_enc.unlock();

    // run controller
    double lin_vel_cmd_fl = velocity_control_fl_.getControlCmd(vel_ref_fl_, vel_enc_fl_);
    double lin_vel_cmd_fr = velocity_control_fr_.getControlCmd(vel_ref_fr_, vel_enc_fr_);
    double lin_vel_cmd_bl = velocity_control_bl_.getControlCmd(vel_ref_bl_, vel_enc_bl_);
    double lin_vel_cmd_br = velocity_control_br_.getControlCmd(vel_ref_br_, vel_enc_br_);
    double steering_vel_cmd_fl = steering_control_fl_.getControlCmd(steer_ref_fl_, steer_enc_fl_);
    double steering_vel_cmd_fr = steering_control_fr_.getControlCmd(steer_ref_fr_, steer_enc_fr_);
    double steering_vel_cmd_bl = steering_control_bl_.getControlCmd(steer_ref_bl_, steer_enc_bl_);
    double steering_vel_cmd_br = steering_control_br_.getControlCmd(steer_ref_br_, steer_enc_br_);


    //steering_vel_cmd_bl = 0.0;
    //steering_vel_cmd_br = 0.0;

    // send commands
    motor_->sendMotorCommands(lin_vel_cmd_fl, lin_vel_cmd_fr,
                              lin_vel_cmd_bl, lin_vel_cmd_br,
                              steering_vel_cmd_fl, steering_vel_cmd_fr,
                              steering_vel_cmd_bl, steering_vel_cmd_br);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "low_level_controller");

    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");

    ros::Duration(1.0).sleep();

    double sampling_time = 0.01;

    LowLevelController lowLevelController(nh, private_nh, sampling_time);

    ros::AsyncSpinner s(2);
    s.start();
    ros::waitForShutdown();

    return 0;
}
