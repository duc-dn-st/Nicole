/*
 * ===============================================================================
 * kinematic.cpp
 * Author: Schaefle Tobias
 * Date: 25.05.2019 -> 21.11.2019 (update)
 * Email: tobias.schaefle@gmail.com
 * -------------------------------------------------------------------------------
 * Description:
 * This file receives encoder data of the wheels and steering shafts and computes
 * the pose and twist of the robot (odom)
 *
 * Subscribes to:
 *
 * Publishes:
 *
 * ===============================================================================
 */

#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_listener.h>
#include <tf/tf.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>

#include <eigen3/Eigen/Core>

#include <mutex>

#include "utilities/utilities.h"

class Kinematic
{
private:

    /// nodehandle
    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_;

    ros::Publisher pubKinematicOdom_;

    ros::Subscriber sub_encoder_;

    //tf::StampedTransform trans_fl_, trans_fr_, trans_bl_, trans_br_;

    Eigen::Vector3d trans_fl_, trans_fr_, trans_bl_, trans_br_;

    nav_msgs::Odometry enc_odom_msg_;

    float heading_;
    Eigen::Vector3f pose_;

    std::mutex encoder_mutex_;

    /// sampling time
    double sampling_time_;
    ros::Timer periodic_timer_;

    // timestamp for pose update
    double prev_timestamp_;

    bool no_encoder_;
    bool simulation_;

    void init()
    {
        pubKinematicOdom_ = private_nh_.advertise<nav_msgs::Odometry>("/encoder_odom", 4);
        if (simulation_)
        {
            //sub_encoder_ = nh_.subscribe("/musashi_robot/joint_states_no_encoder", 1, &Kinematic::encoderCallback, this);
            sub_encoder_ = nh_.subscribe("/musashi_robot/joint_states", 1, &Kinematic::encoderCallback, this);
        }
        else
        {
            sub_encoder_ = nh_.subscribe("/raw_vel", 1, &Kinematic::encoderCallback, this);
        }
        periodic_timer_ = private_nh_.createTimer(ros::Duration(sampling_time_), &Kinematic::publish, this);
    }
public:

    Kinematic(ros::NodeHandle nh, ros::NodeHandle private_nh, float sampling_time)
    {
        this->nh_ = nh;
        this->private_nh_ = private_nh;
        this->sampling_time_ = sampling_time;

        nh.param("no_encoder", no_encoder_, false);
        nh.param("simulation", simulation_, false);

        // set timestamp to current time
        this->prev_timestamp_ = static_cast<float>(ros::Time::now().toSec());

        init();

        enc_odom_msg_.header.frame_id = "odom";

        trans_bl_ << -0.40, 0.29, 0.089;
        trans_br_ << -0.40, -0.29, 0.089;
        trans_fl_ << 0.40, 0.29, 0.089;
        trans_fr_ << 0.40, -0.29, 0.089;

        /*
        tf::TransformListener listener;
        bool fail = true;
        while (fail)
        {
            try
            {
                ros::Duration(1.0).sleep();
                listener.lookupTransform("/base_link", "/back_left_wheel", ros::Time(0), trans_bl_);
                listener.lookupTransform("/base_link", "/back_right_wheel", ros::Time(0), trans_br_);
                listener.lookupTransform("/base_link", "/front_left_wheel", ros::Time(0), trans_fl_);
                listener.lookupTransform("/base_link", "/front_right_wheel", ros::Time(0), trans_fr_);
                fail = false;
            }
            catch (tf::TransformException &ex)
            {
                ROS_ERROR("%s", ex.what());
                ros::Duration(1.0).sleep();
            }
        }
        */

    }

    void encoderCallback(const sensor_msgs::JointStatePtr &encoder_msg)
    {
        std::lock_guard<std::mutex> lock_encoder(encoder_mutex_);

        //std::cout << "Current time:\t" << ros::Time::now() << std::endl;

        float bl_wheel_vel, br_wheel_vel, bl_steer, br_steer,
                fl_wheel_vel, fr_wheel_vel, fl_steer, fr_steer;
        if (simulation_)
        {
            bl_wheel_vel = encoder_msg->velocity[0] * RobotConstants::WHEEL_RADIUS;
            br_wheel_vel = encoder_msg->velocity[1] * RobotConstants::WHEEL_RADIUS;
            bl_steer = GeneralFunctions::wrapTo2Pif(encoder_msg->position[2]);
            br_steer = GeneralFunctions::wrapTo2Pif(encoder_msg->position[3]);
            fl_wheel_vel = encoder_msg->velocity[6] * RobotConstants::WHEEL_RADIUS;
            fr_wheel_vel = encoder_msg->velocity[7] * RobotConstants::WHEEL_RADIUS;
            fl_steer = GeneralFunctions::wrapTo2Pif(encoder_msg->position[4]);
            fr_steer = GeneralFunctions::wrapTo2Pif(encoder_msg->position[5]);
        }
        else
        {
            bl_wheel_vel = encoder_msg->velocity[2] * RobotConstants::WHEEL_RADIUS;
            br_wheel_vel = encoder_msg->velocity[3] * RobotConstants::WHEEL_RADIUS;
            // bl_steer = GeneralFunctions::wrapTo2Pif(encoder_msg->position[4]);
            // br_steer = GeneralFunctions::wrapTo2Pif(encoder_msg->position[6]);
            bl_steer = 0;
            br_steer = 0;
            fl_wheel_vel = encoder_msg->velocity[0] * RobotConstants::WHEEL_RADIUS;
            fr_wheel_vel = encoder_msg->velocity[1] * RobotConstants::WHEEL_RADIUS;
            fl_steer = GeneralFunctions::wrapTo2Pif(encoder_msg->position[5]);
            fr_steer = GeneralFunctions::wrapTo2Pif(encoder_msg->position[7]);
        }



        //std::cout << "Encoder timestamp:\t" << encoder_msg->header.stamp << std::endl;

        Eigen::Vector4f velocities;
        velocities << fl_wheel_vel, fr_wheel_vel, bl_wheel_vel, br_wheel_vel;

        //std::cout << "Driving velocities [fl, fr, bl, br]:\n"
        //          << velocities << std::endl;

        //Eigen::Vector4f angles(fl_steer, fr_steer, bl_steer, br_steer);
        //std::cout << "Steering angles from 0 to 2PI [fl, fr, bl, br]:\n"
        //          << angles << std::endl;

        /*
        // get delta
        float bl_delta = 0.25 * (-trans_bl_.getOrigin().getY() * cosf(bl_steer) + trans_bl_.getOrigin().getX() * sinf(bl_steer)) /
                (trans_bl_.getOrigin().getX() * trans_bl_.getOrigin().getX() + trans_bl_.getOrigin().getY() * trans_bl_.getOrigin().getY());

        float br_delta = 0.25 * (-trans_br_.getOrigin().getY() * cosf(br_steer) + trans_br_.getOrigin().getX() * sinf(br_steer)) /
                (trans_br_.getOrigin().getX() * trans_br_.getOrigin().getX() + trans_br_.getOrigin().getY() * trans_br_.getOrigin().getY());

        float fl_delta = 0.25 * (-trans_fl_.getOrigin().getY() * cosf(fl_steer) + trans_fl_.getOrigin().getX() * sinf(fl_steer)) /
                (trans_fl_.getOrigin().getX() * trans_fl_.getOrigin().getX() + trans_fl_.getOrigin().getY() * trans_fl_.getOrigin().getY());

        float fr_delta = 0.25 * (-trans_fr_.getOrigin().getY() * cosf(fr_steer) + trans_fr_.getOrigin().getX() * sinf(fr_steer)) /
                (trans_fr_.getOrigin().getX() * trans_fr_.getOrigin().getX() + trans_fr_.getOrigin().getY() * trans_fr_.getOrigin().getY());
        */

        // get delta
        float bl_delta = 0.25 * (-trans_bl_.y() * cosf(bl_steer) + trans_bl_.x() * sinf(bl_steer)) /
                (trans_bl_.x() * trans_bl_.x() + trans_bl_.y() * trans_bl_.y());

        float br_delta = 0.25 * (-trans_br_.y() * cosf(br_steer) + trans_br_.x() * sinf(br_steer)) /
                (trans_br_.x() * trans_br_.x() + trans_br_.y() * trans_br_.y());

        float fl_delta = 0.25 * (-trans_fl_.y() * cosf(fl_steer) + trans_fl_.x() * sinf(fl_steer)) /
                (trans_fl_.x() * trans_fl_.x() + trans_fl_.y() * trans_fl_.y());

        float fr_delta = 0.25 * (-trans_fr_.y() * cosf(fr_steer) + trans_fr_.x() * sinf(fr_steer)) /
                (trans_fr_.x() * trans_fr_.x() + trans_fr_.y() * trans_fr_.y());

        Eigen::MatrixXf jacobian(3, 4);
        jacobian << 0.25 * cosf(fl_steer + heading_), 0.25 * cosf(fr_steer + heading_), 0.25 * cosf(bl_steer + heading_), 0.25 * cosf(br_steer + heading_),
                0.25 * sinf(fl_steer + heading_), 0.25 * sinf(fr_steer + heading_), 0.25 * sinf(bl_steer + heading_), 0.25 * sinf(br_steer + heading_),
                fl_delta, fr_delta, bl_delta, br_delta;

        //Eigen::Vector3f state_dot;
        //state_dot = jacobian * velocities;
        //std::cout << "[x_dot, y_dot, theta_dot]:\n"
        //          << state_dot << std::endl;
        float timestamp = static_cast<float>(encoder_msg->header.stamp.toSec());
        //std::cout << timestamp << std::endl;
        //float ns_timestamp = static_cast<float>(encoder_msg->header.stamp.nsec);

        //double timestamp = (double)s_timestamp + (double)ns_timestamp / std::pow(10, 9);


        //float n_time_diff = n_timestamp - n_prev_timestamp_;

        float time_diff = timestamp - prev_timestamp_ ;

        if ( time_diff < 0 )
            time_diff = 0.01;

        if (time_diff > 1)
        {
            ROS_WARN("Unexpected timestamp %f", time_diff);
            time_diff = 0.01;
        }

        //std::cout << velocities << std::endl;
        pose_ = pose_ + (jacobian * velocities) * time_diff;

        enc_odom_msg_.header.stamp = ros::Time::now();

        enc_odom_msg_.pose.pose.position.x = pose_.coeff(0);
        //std::cout <<  time_diff << std::endl;

        enc_odom_msg_.pose.pose.position.y = pose_.coeff(1);
        // update heading
        heading_ = GeneralFunctions::wrapTo2Pif(pose_.coeff(2));
        tf::Quaternion q;
        q.setRPY(0, 0, heading_);
        q.normalize();
        geometry_msgs::Quaternion quat;
        quat.w = q.w();
        quat.x = q.x();
        quat.y = q.y();
        quat.z = q.z();
        enc_odom_msg_.pose.pose.orientation = quat;


        static tf2_ros::TransformBroadcaster br;
        geometry_msgs::TransformStamped transformStamped;

        transformStamped.header.stamp = ros::Time::now();
        transformStamped.header.frame_id = "odom";
        transformStamped.child_frame_id = "base_link";
        transformStamped.transform.translation.x = enc_odom_msg_.pose.pose.position.x;
        transformStamped.transform.translation.y = enc_odom_msg_.pose.pose.position.y;
        transformStamped.transform.translation.z = 0.0;
        //tf::Quaternion q;
        q.setRPY(0, 0, heading_);
        transformStamped.transform.rotation.x = q.x();
        transformStamped.transform.rotation.y = q.y();
        transformStamped.transform.rotation.z = q.z();
        transformStamped.transform.rotation.w = q.w();

        br.sendTransform(transformStamped);

        //std::cout << "Heading:\t" << heading_ << std::endl;

        // reset prev timestamp
        prev_timestamp_ = timestamp;
    }

    void publish(const ros::TimerEvent &event)
    {
        std::lock_guard<std::mutex> lock_encoder(encoder_mutex_);
        pubKinematicOdom_.publish(enc_odom_msg_);
    }

};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "kinematics");
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");

    float sampling_time = 0.02f; //[s]

    Kinematic kinematic(nh, private_nh, sampling_time);

    ros::AsyncSpinner s(2);
    s.start();
    ros::waitForShutdown();

    return 0;
}

