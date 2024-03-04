/*
 * ===============================================================================
 * pure_pursuit.cpp
 * Author: Schaefle Tobias
 * Date: 07.09.20
 * Email: tobias.schaefle@gmail.com
 * -------------------------------------------------------------------------------
 * Description:
 * A temporary controller for the ackerman drive setup.
 * ===============================================================================
 */

#include "../include/control/open_loop.h"

void OpenLoop::control(const Eigen::Vector3d &robot_pose, const Eigen::Vector3d &pose_ref, const double linear_vel_ref, const double angular_vel_ref,
                       const double &wheel_enc_fl, const double &wheel_enc_fr, const double &wheel_enc_bl, const double &wheel_enc_br,
                       const double &steering_enc_fl, const double &steering_enc_fr, const double &steering_enc_bl, const double &steering_enc_br)
{
    // calculate the errors
    Eigen::Vector3d error = pose_ref - robot_pose;

    // check if at goal
    if (atGoal(error))
    {
        //stopMotion();
        return;
    }

    //if (GeneralFunctions::isEqual(angular_vel_ref, 0.0))
    if (std::abs(angular_vel_ref) < 0.001)
    {
        double vel = linear_vel_ref / RobotConstants::WHEEL_RADIUS;
        std::cout << "Wheel velocity:\t" << vel << std::endl;
        sendCommands(vel, vel, vel, vel, 0.0, 0.0, 0.0, 0.0,
                     steering_enc_fl, steering_enc_fr, steering_enc_bl, steering_enc_br);
    }
    else
    {
        double turn_radius = linear_vel_ref / std::abs(angular_vel_ref);
        double outer_steering = std::atan(RobotConstants::FRONT_TO_REAR_WHEEL / (turn_radius + RobotConstants::AXLE_LENGTH / 2));
        double inner_steering = std::atan(RobotConstants::FRONT_TO_REAR_WHEEL / (turn_radius - RobotConstants::AXLE_LENGTH / 2));
        double outer_vel = (linear_vel_ref + RobotConstants::AXLE_LENGTH * std::abs(angular_vel_ref) / 2) / RobotConstants::WHEEL_RADIUS;
        double inner_vel = (linear_vel_ref - RobotConstants::AXLE_LENGTH * std::abs(angular_vel_ref) / 2) / RobotConstants::WHEEL_RADIUS;

        // check which wheels are on the inside of turning
        std::cout << "omega:\t" << angular_vel_ref << std::endl;
        if (angular_vel_ref < 0)
        {
            // right turn
            //outer_steering = RobotLimits::steeringLimit(steering_enc_fl, outer_steering, sampling_time_);
            //inner_steering = RobotLimits::steeringLimit(steering_enc_fr, inner_steering, sampling_time_);
            //outer_vel = RobotLimits::drivingLimit(wheel_enc_fl, outer_vel, sampling_time_);
            //inner_vel = RobotLimits::drivingLimit(wheel_enc_fr, inner_vel, sampling_time_);
            sendCommands(outer_vel, inner_vel, outer_vel, inner_vel,
                         -outer_steering, -inner_steering, 0.0, 0.0,
                         steering_enc_fl, steering_enc_fr, steering_enc_bl, steering_enc_br);
            std::cout << "Outer wheel velocity and steering:\t[" << outer_vel << ", " << -outer_steering << "]" << std::endl;
            std::cout << "Inner wheel velocity and steering:\t[" << inner_vel << ", " << -inner_steering << "]" << std::endl;
        }
        else
        {
            // left turn
            //outer_steering = RobotLimits::steeringLimit(steering_enc_fr, outer_steering, sampling_time_);
            //inner_steering = RobotLimits::steeringLimit(steering_enc_fl, inner_steering, sampling_time_);
            //outer_vel = RobotLimits::drivingLimit(wheel_enc_fr, outer_vel, sampling_time_);
            //inner_vel = RobotLimits::drivingLimit(wheel_enc_fl, inner_vel, sampling_time_);
            sendCommands(inner_vel, outer_vel, inner_vel, outer_vel,
                         inner_steering, outer_steering, 0.0, 0.0,
                         steering_enc_fl, steering_enc_fr, steering_enc_bl, steering_enc_br);
            std::cout << "Outer wheel velocity and steering:\t[" << outer_vel << ", " << outer_steering << "]" << std::endl;
            std::cout << "Inner wheel velocity and steering:\t[" << inner_vel << ", " << inner_steering << "]" << std::endl;
        }

    }
}
