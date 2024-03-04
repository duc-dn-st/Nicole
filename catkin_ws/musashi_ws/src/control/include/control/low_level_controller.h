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

#ifndef LOW_LEVEL_CONTROLLER_H
#define LOW_LEVEL_CONTROLLER_H

#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <dynamic_reconfigure/server.h>

#include <mutex>
#include <thread>

#include "sdv_msgs/ControlReference.h"
#include "drivers/motor_driver.h"
#include "control/ControlGainsConfig.h"

class VelocityController
{
public:
    VelocityController() {}

    VelocityController(double sampling_time)
    {
        sampling_time_ = sampling_time;
        integral_err_ = 0.0;
        prev_err_;
        k_p_ = 0.05;
        k_i_ = 0.01;
    }

    void updateGains(double k_p, double k_i) {k_p_ = k_p; k_i_, k_i;}

    double getControlCmd(const double vel_ref, const double vel);

private:
    double sampling_time_;
    double integral_err_;
    double prev_err_;
    double k_p_, k_i_;
};

class SteeringController
{
public:
    SteeringController() {}

    SteeringController(double sampling_time)
    {
        sampling_time_ = sampling_time;
        integral_err_ = 0.0;
        prev_err_;
        k_p_ = 8;
        k_i_ = 0.05;
    }

    void updateGains(double k_p, double k_i) {k_p_ = k_p; k_i_, k_i;}

    double getControlCmd(const double steering_ref, const double steering);

private:
    double sampling_time_;
    double integral_err_;
    double prev_err_;
    double k_p_, k_i_;
};

class LowLevelController
{
public:
    LowLevelController(ros::NodeHandle nh, ros::NodeHandle Private_nh, double sampling_time)
    {
        nh_ = nh;
        private_nh_ = Private_nh;

        sampling_time_ = sampling_time;

        motor_ = new MotorDriver;
        motor_->openDAC();

        encoder_ = nullptr;

        sub_encoder_ = nh_.subscribe("/raw_vel", 1, &LowLevelController::encoderCallback, this);
        sub_reference_ = nh_.subscribe("/reference", 1, &LowLevelController::controlReferenceCallback, this);

        periodic_timer_ = private_nh_.createTimer(ros::Duration(sampling_time_), &LowLevelController::runController, this);

        dynamic_reconfigure::Server<control::ControlGainsConfig> server;
        dynamic_reconfigure::Server<control::ControlGainsConfig>::CallbackType f;
        f = boost::bind(&LowLevelController::gainCallback, this, _1, _2);
        server.setCallback(f);

        velocity_control_fl_ = VelocityController(sampling_time);
        velocity_control_fr_ = VelocityController(sampling_time);
        velocity_control_bl_ = VelocityController(sampling_time);
        velocity_control_br_ = VelocityController(sampling_time);

        steering_control_fl_ = SteeringController(sampling_time);
        steering_control_fr_ = SteeringController(sampling_time);
        steering_control_bl_ = SteeringController(sampling_time);
        steering_control_br_ = SteeringController(sampling_time);

        // initial reference
        reference_.velocity_ref_front_left = 0.0;
        reference_.velocity_ref_front_right = 0.0;
        reference_.velocity_ref_back_left = 0.0;
        reference_.velocity_ref_back_right = 0.0;
        reference_.steering_ref_front_left = 0.0;
        reference_.steering_ref_front_right = 0.0;
        reference_.steering_ref_back_left = 0.0;
        reference_.steering_ref_back_right = 0.0;
    }

    ~LowLevelController()
    {
        motor_->closeDAC();
    }

    void controlReferenceCallback(const sdv_msgs::ControlReferenceConstPtr ref_msg);

    void encoderCallback(const sensor_msgs::JointStatePtr encoder_msg);

    void runController(const ros::TimerEvent &event);

    void gainCallback(control::ControlGainsConfig &config, uint32_t level);

private:
    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_;

    ros::Timer periodic_timer_;

    ros::Subscriber sub_encoder_;
    ros::Subscriber sub_reference_;

    VelocityController velocity_control_fl_;
    VelocityController velocity_control_fr_;
    VelocityController velocity_control_bl_;
    VelocityController velocity_control_br_;

    SteeringController steering_control_fl_;
    SteeringController steering_control_fr_;
    SteeringController steering_control_bl_;
    SteeringController steering_control_br_;

    double vel_ref_fl_, vel_ref_fr_, vel_ref_bl_, vel_ref_br_;
    double steer_ref_fl_, steer_ref_fr_, steer_ref_bl_, steer_ref_br_;

    double vel_enc_fl_, vel_enc_fr_, vel_enc_bl_, vel_enc_br_;
    double steer_enc_fl_, steer_enc_fr_, steer_enc_bl_, steer_enc_br_;

    sdv_msgs::ControlReference reference_;
    sensor_msgs::JointStatePtr encoder_;

    std::mutex reference_mutex_, encoder_mutex_;

    MotorDriver *motor_;

    double sampling_time_;
};

#endif // LOW_LEVEL_CONTROLLER_H
