/*
 * ===============================================================================
 * simple_controller.cpp
 * Author: Schaefle Tobias
 * Date: 29.06.20
 * Email: tobias.schaefle@gmail.com
 * -------------------------------------------------------------------------------
 * Description:
 * First controller for the SDV. It receives the robot pose a reference point and
 * encoder values. Controller commands will be calculated and send to the motors.
 * ===============================================================================
 */

#include "../include/control/simple_controller.h"

/**
 * @brief 
 * 
 * @param transform transform to current wheel
 * @param cos_theta cos at current heading
 * @param sin_theta sin at current heading
 * @param steering_angle current steering angle
 * @param e_x error in x direction
 * @param e_y error in y direction
 * @param e_heading error in the heading
 * @param x_dot_ref reference vel in x
 * @param y_dot_ref reference vel in y
 * @param heading_rate_radps_ref reference heading change
 * @param beta 
 * @param gamma 
 * @param alpha_err 
 */
void SimpleController::getControlParams(tf::StampedTransform transform, const double cos_theta, const double sin_theta, const double enc_steering,
                         const Eigen::Vector3d &error, const  Eigen::Vector3d &twist_ref,
                         double &beta, double &gamma, double &alpha_err) const
{
    double v_refx = twist_ref(0) * cos_theta + twist_ref(1) * sin_theta
            - transform.getOrigin().getY() * twist_ref(2);
    double v_refy = twist_ref(1) * cos_theta - twist_ref(0) * sin_theta
            + transform.getOrigin().getX() * twist_ref(2);
    double v_ref = sqrt(v_refx * v_refx + v_refy * v_refy);

    double angle_vx_vy = atan2(v_refy, v_refx);
    
    double alpha_ref = angle_vx_vy;
    beta = cos(alpha_ref) * v_ref + Kx * error(0) - Kt * transform.getOrigin().getY() * error(2);
    gamma = sin(alpha_ref) * v_ref + Ky * error(1) + Kt * transform.getOrigin().getX() * error(2);
    alpha_err = std::abs(alpha_ref - enc_steering);
}

/**
 * @brief 
 * 
 * @param alpha_err 
 * @param beta 
 * @param gamma 
 * @param enc_vel current wheel vel from encoder
 * @param wheel_cmd wheel vel in [m/s] 
 */
void SimpleController::getWheelVelCmd(const double alpha_err, const double beta, const double gamma, const double enc_vel, double &wheel_cmd) const
{
    if (alpha_err > MathConstants::PI / 2)
    {
        wheel_cmd = -sqrt(beta * beta + gamma * gamma);
    }
    else
    {
        wheel_cmd = sqrt(beta * beta + gamma * gamma);
    }
    // check input constraints
    wheel_cmd = RobotLimits::drivingLimit(enc_vel, wheel_cmd, sampling_time_);
    // convert to rad/s
    wheel_cmd = wheel_cmd / RobotConstants::WHEEL_RADIUS;
}

/**
 * @brief 
 * 
 * @param alpha_err 
 * @param beta 
 * @param gamma 
 * @param enc_steering current steering angle from encoder
 * @param steering_cmd steering command in [rad]
 */
void SimpleController::getSteeringCmd(const double alpha_err, const double beta, const double gamma, const double enc_steering, double &steering_cmd) const
{
    if (alpha_err > MathConstants::PI / 2)
    {
        steering_cmd = GeneralFunctions::wrapTo2Pi(MathConstants::PI + atan2(gamma, beta));
    }
    else
    {
        steering_cmd = atan2(gamma, beta);
    }
    // check input constraints
    steering_cmd = RobotLimits::steeringLimit(enc_steering, steering_cmd, sampling_time_);

}

void SimpleController::control(const Eigen::Vector3d &robot_pose, const Eigen::Vector3d &pose_ref, const Eigen::Vector3d &twist_ref, 
                const double &wheel_enc_fl, const double &wheel_enc_fr, const double &wheel_enc_bl, const double &wheel_enc_br, 
                const double &steering_enc_fl, const double &steering_enc_fr, const double &steering_enc_bl, const double &steering_enc_br)
{
    std::cout << "Call simple controller" << std::endl;
    double wheel_cmd_fl, wheel_cmd_fr, wheel_cmd_bl, wheel_cmd_br;
    double steering_cmd_fl, steering_cmd_fr, steering_cmd_bl, steering_cmd_br;

    // calculate the errors
    Eigen::Vector3d error = pose_ref - robot_pose;

    // check if at goal
    if (atGoal(error))
    {
        std::cout << "At goal" << std::endl;
        stopMotion();
        return;
    }

    // transform to robot coordinates
    double e_robotx = error(0) * cos(robot_pose(2)) + error(1) * sin(robot_pose(2));
    double e_roboty = error(0) * sin(robot_pose(2)) + error(1) * cos(robot_pose(2));

    double cos_theta = cos(pose_ref(2));
    double sin_theta = sin(pose_ref(2));

    double beta, gamma, alpha_err;

    // get control commands for fl
    getControlParams(trans_fl_, cos_theta, sin_theta, steering_enc_fl, error, twist_ref, beta, gamma, alpha_err);
    getWheelVelCmd(alpha_err, beta, gamma, wheel_enc_fl, wheel_cmd_fl);
    getSteeringCmd(alpha_err, beta, gamma, steering_enc_fl, steering_cmd_fl);
    // get control commands for fr
    getControlParams(trans_fr_, cos_theta, sin_theta, steering_enc_fr, error, twist_ref, beta, gamma, alpha_err);
    getWheelVelCmd(alpha_err, beta, gamma, wheel_enc_fr, wheel_cmd_fr);
    getSteeringCmd(alpha_err, beta, gamma, steering_enc_fr, steering_cmd_fr);
    // get control commands for bl
    getControlParams(trans_bl_, cos_theta, sin_theta, steering_enc_bl, error, twist_ref, beta, gamma, alpha_err);
    getWheelVelCmd(alpha_err, beta, gamma, wheel_enc_bl, wheel_cmd_bl);
    getSteeringCmd(alpha_err, beta, gamma, steering_enc_bl, steering_cmd_bl);
    // get control commands for br
    getControlParams(trans_br_, cos_theta, sin_theta, steering_enc_br, error, twist_ref, beta, gamma, alpha_err);
    getWheelVelCmd(alpha_err, beta, gamma, wheel_enc_br, wheel_cmd_br);
    getSteeringCmd(alpha_err, beta, gamma, steering_enc_br, steering_cmd_br);

    // send commands to actuator
    sendCommands(wheel_cmd_fl, wheel_cmd_fr, wheel_cmd_bl, wheel_cmd_br,
                steering_cmd_fl, steering_cmd_fr, steering_cmd_bl, steering_cmd_br,
                steering_enc_fl, steering_enc_fr, steering_enc_bl, steering_enc_br);
}