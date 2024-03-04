/*
 * ===============================================================================
 * obstacle_avoidance.cpp
 * Author: Weidmann Daniel
 * Date: 21.01.2020
 * Email: weidmann.da@gmail.com
 * -------------------------------------------------------------------------------
 * Description:
 * This file implements the vector field histogram algorithm to avoid obstacles.
 *
 * Subscribes to:
 *      /pc_to_local_map
 * Publishes:
 *      /polar_histogram
 * ===============================================================================
 */

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/OccupancyGrid.h>
#include <tf/transform_datatypes.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <nav_msgs/Path.h>

#include <Eigen/Dense>

#include <algorithm>
#include <cmath>
#include <fstream>
#include <mutex>
#include <vector>

#include <boost/math/constants/constants.hpp>

#include "sdv_msgs/Trajectory.h"
#include "sdv_msgs/TrajectoryPoint.h"
#include "utilities/utilities.h"

constexpr double PI = boost::math::constants::pi<double>();

class ObstacleAvoidance
{
private:
    // General
    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_;

    ros::Publisher local_traj_pub_;
    ros::Publisher polar_hist_pub_;
    ros::Publisher heading_vector_pub_;
    ros::Publisher heading_path_pub_;

    ros::Subscriber odom_sub_;
    ros::Subscriber costmap_sub_;
    ros::Subscriber trajectory_sub_;
    ros::Subscriber goal_sub_;

    sdv_msgs::Trajectory local_traj_msg_;
    sdv_msgs::TrajectoryPoint next_local_pt_;
    nav_msgs::OccupancyGrid histogram_grid_msg_;
    geometry_msgs::PoseStamped heading_vector_msg_;
    nav_msgs::Path heading_path_msg_;

    const float HIST_RESOLUTION = 0.05;
    const unsigned int HIST_WIDTH = 200;
    const unsigned int HIST_HEIGHT = 200;

    // Polar histogram
    double alpha_;                  // angular resolution (in rad)
    double threshold_;              // threshold for sector to be passable
    double param_a_, param_b_;      // parameters for the calculation of obstacle vector magnitude
    int s_max_;
    double d_max_;
    double controller_sampling_time_;
    std::vector<double> densities_; // obstacle density per sector

    Eigen::Vector2d goal_;

    // Robot pose
    double x_pos_, y_pos_;
    float heading_;
    double roll_, pitch_, yaw_, angular_vel_;

    double heading_angle_;

    // sampling time
    double sampling_time_;
    ros::Timer periodic_timer_;

    std::mutex costmap_mutex_, odom_mutex_, trajectory_mutex_, goal_mutex_;


    void updateHistogram(const nav_msgs::OccupancyGrid::ConstPtr &costmap)
    {
        if (costmap != nullptr)
        {
            //ROS_WARN("Received new costmap");
            // clear the old histogram
            for (int ii = 0; ii < densities_.size(); ++ii)
            {
                densities_[ii] = 0;
            }
            /////////////TURN OFF MAP//////////////
            if (Testing::TRACKING_ONLY)
            {
                return;
            }
            ///////////////////////////////////////

            // Position of robot inside the map
            double X = costmap->info.width / 2;
            double Y = costmap->info.height / 2;

            // maximum distance considered (only consider obstacles in a circle around robot)
            //double max_dist = costmap->info.width * costmap->info.resolution / 2;

            // calculate obstacle vector for every cell in the costmap
            // use this obstacle vector to calculate obstacle density for every sectors
            for (unsigned int i = 0; i < costmap->info.width; ++i)
            {
                for (unsigned int j = 0; j < costmap->info.height; ++j)
                {
                    double x_pos = (i - X) * costmap->info.resolution;
                    double y_pos = (j - Y) * costmap->info.resolution;
                    // angular position of this cell
                    // equation (1)
                    double beta = atan2(y_pos, x_pos);
                    beta = GeneralFunctions::wrapTo2Pi(beta);

                    //std::cout << "[x, y] = " << x_pos << ", " << y_pos << std::endl;

                    // magnitude of this cell
                    double dist = sqrt(x_pos * x_pos + y_pos * y_pos); // Euclidian distance from this cell to robot

                    if (dist > d_max_)
                    {
                        continue; // this cell is not considered in building the histogram
                    }

                    double magn = 0.0;
                    if (costmap->data[i + j * costmap->info.width] >= 50.0 && costmap->data[i + j * costmap->info.width] <= 110.0)
                    {
                        //magn = 1.0;
                        // from inflation map
                        // magn is either 1, 075 or 0.5
                        magn = costmap->data[i + j * costmap->info.width] / 100;
                    }
                    magn = magn * magn;
                    // equation (2)
                    magn *= param_a_ - param_b_ * dist;
                    // equation (3)
                    int sector = static_cast<int>(beta / alpha_) % densities_.size();
                    //int sector = floor((beta + PI) / alpha_);
                    //ROS_INFO("Distance and Angle: %f, %f", dist, sector * alpha_ * 180 / PI);
                    // equation (4)
                    densities_[sector] += magn;
                }
            }

            // smooth the densities : equation (5)
            int l = 5;
            std::vector<double> smoothed_densities = std::vector<double>(densities_.size());
            for (int i = 0; i < densities_.size(); ++i)
            {
                double smoothed_density = 0;
                for (int j = -l + 1; j < l; ++j)
                {
                    int index = (i + j + densities_.size()) % densities_.size();
                    smoothed_density += abs(l - j) * densities_[index];
                }
                smoothed_densities[i] = smoothed_density / (2 * (double)l + 1.0);
                //ROS_INFO("Normal Density:\t%f: %f", i * alpha_ * 180 / PI, densities_[i]);
                //ROS_INFO("Smoothed Density:\t%f: %f", i * alpha_ * 180 / PI, smoothed_densities[i]);
            }
            densities_ = smoothed_densities;
        }
    }


    /*!
    * Get a representation of the polar histogram as OccupancyGrid
    * \brief PolarHistogram::toOccupancyGrid
    * \param resolution the resolution of the map
    * \param width, height the size of the map
    * \return an OccupancyGrid representation of the polar histogram
    */
    void updateHistogramMsg()
    {
        int X = HIST_WIDTH / 2;
        int Y = HIST_HEIGHT / 2;

        // ROS_WARN("Max element is %f", max_density);
        // ROS_WARN("Drawing polar histogram");

        histogram_grid_msg_.info.origin.position.x = -static_cast<double>(HIST_RESOLUTION) * HIST_WIDTH / 2 + x_pos_;
        histogram_grid_msg_.info.origin.position.y = -static_cast<double>(HIST_RESOLUTION) * HIST_HEIGHT / 2 + y_pos_;

        if (!(polar_hist_pub_.getNumSubscribers() < 1))
        {
            double max_density = *max_element(densities_.begin(), densities_.end());
            std::cout << "Max Density: " << max_density << std::endl;

            // visibility of histogram
            double convert_val;
            double add_val = 25.0;;
            if (max_density > 0.0)
            {
                convert_val = 75 / max_density;
            }
            else
            {
                convert_val = 1.0;
            }

            // Fill the map, each cell belongs to one of the sectors of the polar histogram
            for (int i = 0; i < HIST_WIDTH; ++i)
            {
                for (int j = 0; j < HIST_HEIGHT; ++j)
                {
                    // Determine sector of this cell
                    double beta = atan2( static_cast<double>(j - Y), static_cast<double>(i - X) );
                    beta = GeneralFunctions::wrapTo2Pi(beta);
                    int sector = static_cast<int>(beta / alpha_) % densities_.size();
                    //int sector = (int)floor( (beta + PI) / alpha_ ) % densities_.size();
                    if (sector < 0 || sector > 360) ROS_ERROR("Illegal sector!"); //143


                    int dens = densities_[sector] * convert_val > 0 ? densities_[sector] * convert_val + add_val : densities_[sector] * convert_val;

                    /*
                    double dens = densities_[sector] * 100.0;

                    if (max_density != 0.0) dens = dens / max_density;

                    if (dens < 1.0) { dens = 0.0; }
                    else if (dens < 20.0) { dens = 15.0; }
                    else if (dens < 40.0) { dens = 30.0; }
                    else if (dens < 60.0) { dens = 45.0; }
                    else if (dens < 80.0) { dens = 60.0; }
                    else { dens = 75.0; }
                    */
                    histogram_grid_msg_.data[i + j * HIST_WIDTH] = dens;
                    // ROS_INFO("%f", dens);
                }
            }
            polar_hist_pub_.publish(histogram_grid_msg_);
        }
        /*
        // clear entire grid
        for (size_t i = 0; i < HIST_WIDTH; ++i)
        {
            for (int j = 0; j < HIST_HEIGHT; ++j)
            {
                histogram_grid_msg_.data[i + j * HIST_WIDTH] = 0.0;
            }
        }
        // ROS_INFO("Besenham");
        // Plot a straight line with value 100 for the target direction
        double steeringAngle = local_traj_msg_.points.back().heading;
        // ROS_WARN("%f", steeringAngle);

        // Bresenham's line algorithm (https://en.wikipedia.org/wiki/Bresenham%27s_line_algorithm)
        double radius = histogram_grid_msg_.info.width / 2;
        int x0 = histogram_grid_msg_.info.width / 2;
        int y0 = histogram_grid_msg_.info.height / 2;
        int x1 = cos(steeringAngle) * radius + x0;
        int y1 = sin(steeringAngle) * radius + y0;

        int dx = abs(x1 - x0);
        int sx = x0 < x1? 1 : -1;
        int dy = -abs(y1 - y0);
        int sy = y0 < y1? 1 : -1;
        int err = dx + dy;
        while (true) {
            histogram_grid_msg_.data[x0 + y0 * HIST_WIDTH] = 100.0;   // "plot"
            if (x0 == x1 && y0 == y1) break;
            int e2 = 2*err;
            if (e2 >= dy) {
                err += dy;
                x0 += sx;
            }
            if (e2 <= dx) {
                err += dx;
                y0 += sy;
            }
        }

            // ROS_INFO("Besenham done");

        */
    }


    /*!
    * Calculate the steering angle from the polar histogram
    * \brief PolarHistogram::calcDirection
    * \param currentPos the coordinates of the robot
    * \param laPoint the coordinates of the target
    * \return the steering angle to avoid obstacles
    */
    void calcDirectionAndDensity(Eigen::Vector2d &currentPos, Eigen::Vector2d &laPoint, double &steering_angle, double &density)
    {
        // get sector in which the target lies
        int targetSector = laSector(currentPos, laPoint);

        // find valleys, possibly close to the target sector
        std::vector<int> valleys;
        std::vector<double> angles;
        findValleys(targetSector, valleys, angles);

        int nearest = 0;

        double min_rel_angle;

        double point_angle;

        if (valleys.empty())
        {
            steering_angle = 0;
            point_angle = 0.0;
            density = 999;
            ROS_WARN("OBSTACLE_AVOIDANCE_WARN: No safe valley!");
            //return;
        }
        else if (valleys.size() == 1)
        {
            nearest = valleys.at(0);
            point_angle = angles.at(0);
        }
        else
        {
            nearest = valleys.at(0);
            point_angle = angles.at(0);
            double angle;
            // calculate angle
            min_rel_angle = GeneralFunctions::absoluteDifferenceBetweenAngles(nearest * alpha_, targetSector * alpha_);
            for (int ii = 1; ii < valleys.size(); ++ii)
            {
                angle = GeneralFunctions::absoluteDifferenceBetweenAngles(valleys.at(ii) * alpha_, targetSector * alpha_);
                if (angle < min_rel_angle)
                {
                    nearest = valleys.at(ii);
                    point_angle = angles.at(ii);
                    min_rel_angle = angle;
                }
            }
        }
        /*
        bool valley_exists = false;
        for (int const valley : valleys)
        {
            if (abs(valley - targetSector) < abs(nearest - targetSector))
            {
                nearest = valley;
                valley_exists = true;
            }
        }
        if (!valley_exists)
        {
            steering_angle = 0;
            density = 999;
            ROS_WARN("OBSTACLE_AVOIDANCE_WARN: No safe valley!");
            return;
        }
        */

        density = densities_[nearest];

        double steering;
        if (nearest == targetSector) {      // if target sector is the nearest, directly steer towards the target angle
            steering = atan2(laPoint[1] - currentPos[1], laPoint[0] - currentPos[0]);
        } else {        // else steer towards the center of the nearest valley
            //steering = static_cast<int>(nearest) * alpha_ + 0.5*alpha_ + PI;
            //steering = nearest * alpha_;
            steering = point_angle;
        }
        steering = GeneralFunctions::wrapTo2Pi(steering);
        steering_angle = steering;
    }



    // private helper funtions

    /*!
    * Calculate the sector that matches the target direction
    * \brief PolarHistogram::laSector
    * \param currentPos the coordinates of the robot
    * \param laPoint the coordinates of the target
    * \return index of the sector that matches the target direction
    */
    inline int laSector(Eigen::Vector2d &currentPos, Eigen::Vector2d &laPoint) const
    {
        double laAngle = atan2(laPoint[1] - currentPos[1], laPoint[0] - currentPos[0]);
        laAngle = GeneralFunctions::wrapTo2Pi(laAngle);
        return static_cast<int>(laAngle / alpha_) % densities_.size();
    }


    /*!
    * Calculate all candidate valleys for the robot to pass through
    * \brief PolarHistogram::findValleys
    * \param targetSector the sector of the target position
    * \return a vector of all candidate valleys
    */
    void findValleys(int targetSector, std::vector<int> &valleys, std::vector<double> &angles)
    {
        angles.clear();
        double angle;
        // (copied from Matthew's code)
        int k = 0, j = 0, k1, k2, kn, kf;
        double k1_angle, k2_angle, kn_angle, kf_angle;
        //int smax = 15; //width of the valley
        std::vector<int> candidates;
        while (k <= densities_.size())
        {
            if (densities_[k] < threshold_)
            {
                k1 = k; // right border of a valley
                k2 = k1;
                while (k <= densities_.size() && densities_[k]< threshold_)
                {
                    k2 = k; // left border of a valley
                    k = k + 1;
                }

                if (k2 == k1)
                {
                    continue;
                }

                k1_angle = k1 * alpha_;
                k2_angle = k2 * alpha_;

                // check if target sector belongs inside valley
                if (k1 <=  targetSector && k2 >= targetSector)
                {
                    candidates.push_back(targetSector);
                    // not needed to calculate
                    angles.push_back(0.0);
                    // no need to search other candidates
                    valleys = candidates;
                    return;
                }

                // check if wide valley
                if ((k2 - k1) > s_max_)
                {
                    // find kn and kf (page 283)
                    // kn can only be either k1 or k2
                    double k1_dist = GeneralFunctions::wrapToPi(targetSector * alpha_ - k1_angle);
                    double k2_dist = GeneralFunctions::wrapToPi(targetSector * alpha_ - k2_angle);

                    if (std::abs(k2_dist) < std::abs(k1_dist))
                    {
                        kn = k2;
                        kn_angle = k2_angle;
                        kf = (kn - s_max_) % densities_.size();
                    }
                    else
                    {
                        kn = k1;
                        kn_angle = k1_angle;
                        kf = (kn + s_max_) % densities_.size();
                    }

                    kf_angle = kf * alpha_;
                    // get sector
                    int sector_candidate;
                    double rel_angle = GeneralFunctions::absoluteDifferenceBetweenAngles(kf_angle, kn_angle) / 2;
                    if (kn < kf)
                    {
                        angle = GeneralFunctions::wrapTo2Pi(kn_angle + rel_angle);
                        sector_candidate = static_cast<int>(angle / alpha_) % densities_.size();
                    }
                    else
                    {
                        angle = GeneralFunctions::wrapTo2Pi(kn_angle - rel_angle);
                        sector_candidate = static_cast<int>(angle / alpha_) % densities_.size();
                    }
                    candidates.push_back(sector_candidate);
                    angles.push_back(angle);
                }
                else
                {
                    // narrow valley -> not much explanation in paper
                    // but basically same equations as in wide valley, except kf
                    // find kn and kf (page 283)
                    // kn can only be either k1 or k2
                    double k1_dist = GeneralFunctions::wrapToPi(targetSector * alpha_ - k1 * alpha_);
                    double k2_dist = GeneralFunctions::wrapToPi(targetSector * alpha_ - k2 * alpha_);

                    if (std::abs(k2_dist) < std::abs(k1_dist))
                    {
                        kn = k2;
                        kf = k1;
                        kn_angle = k2_angle;
                        kf_angle = k1_angle;
                    }
                    else
                    {
                        kn = k1;
                        kf = k2;
                        kn_angle = k1_angle;
                        kf_angle = k2_angle;
                    }
                    // get sector
                    int sector_candidate;
                    double rel_angle = GeneralFunctions::absoluteDifferenceBetweenAngles(kf_angle, kn_angle) / 2;
                    if (kn < kf)
                    {
                        angle = GeneralFunctions::wrapTo2Pi(kn_angle + rel_angle);
                        sector_candidate = static_cast<int>(angle / alpha_) % densities_.size();
                    }
                    else
                    {
                        angle = GeneralFunctions::wrapTo2Pi(kn_angle - rel_angle);
                        sector_candidate = static_cast<int>(angle / alpha_) % densities_.size();
                    }
                    candidates.push_back(sector_candidate);
                    angles.push_back(angle);
                }

                /*
                // calculate candidate directions :
                if (kl - kr > s_max_) // wide valley
                {
                    candidates.push_back(static_cast<int>(kl - s_max_ / 2)); // towards the left side
                    candidates.push_back(static_cast<int>(kr + s_max_ / 2)); // towards the right side
                    j = j + 2;
                    if (targetSector >= kr && targetSector <= kl)
                    {
                        candidates.push_back(targetSector); // straight at look ahead
                        j = j + 1;
                    }
                }
                else if(kl - kr > s_max_ / 5) // narrow valley
                {
                    candidates.push_back(static_cast<int>((kr + kl) / 2));
                    j = j + 1;
                }
                */
            }
            else
            {
                    k = k + 1;
            }
        }

        valleys = candidates;
    }

    /**
     * @brief createLocalTrajectory
     */
    void createLocalTrajectory()
    {
        Eigen::Vector2d currentPos(x_pos_, y_pos_);
        double density, steeringAngle;
        //goal_ << 1, 1;
        calcDirectionAndDensity(currentPos, goal_, steeringAngle, density);
        heading_angle_ = steeringAngle;
        if (-2*PI <= steeringAngle && steeringAngle <= 2*PI)
        {
            // initialize values
            local_traj_msg_.points.clear();
            next_local_pt_.heading = yaw_;//steeringAngle;
            next_local_pt_.x = x_pos_;
            next_local_pt_.y = y_pos_;
            next_local_pt_.x_dot = 0.0; // cos(steeringAngle) * 0.1;
            next_local_pt_.y_dot = 0.0; // sin(steeringAngle) * 0.1;
            next_local_pt_.heading_rate_radps = 0.0;
            next_local_pt_.velocity_mps = 0.0;
            next_local_pt_.acceleration_mps2 = 0.0;
            next_local_pt_.heading_acc_radps2 = 0.0;

            // angular velocity
            double ang_vel = GeneralFunctions::angularVelocity(yaw_, steeringAngle, controller_sampling_time_);
            //std::cout << "Angular velocity:" << ang_vel << std::endl;
            // check if it is above self defined constraint
            double max_ang_vel = 10 * PI / 180;
            if (std::abs(ang_vel) < max_ang_vel)
            {
                next_local_pt_.heading_rate_radps = ang_vel;
            }
            else
            {
                next_local_pt_.heading_rate_radps = (ang_vel < 0) ? -max_ang_vel : max_ang_vel;
            }

            //std::cout << "Angular velocity after constraint:" << next_local_pt_.heading_rate_radps << std::endl;

            // linear velocity
            double lin_vel;// = TrajectoryParameters::PATH_VEL_LIM;
            // use equation 6 and 7 of the paper
            // get the density entry at the current robot heading and calculate the recommended velocity
            double density = densities_[static_cast<unsigned int>(yaw_ / alpha_) % densities_.size()];
            // velocity generayion
            lin_vel = std::max(TrajectoryParameters::PATH_VEL_MIN, TrajectoryParameters::PATH_VEL_LIM * (1 - std::min(density, ControlConstants::VFH_DENSITY_CONSTANT) / ControlConstants::VFH_DENSITY_CONSTANT));
            std::cout << "VFH Linear Velcoity: " << lin_vel << std::endl;
            // calculate linear velocity based on steering rate and density of the area


            // generate horizon length trajectory points with new heading
            for (unsigned int ii = 0; ii < ControlConstants::HORIZON; ++ii)
            {
                if (!(std::abs(steeringAngle - next_local_pt_.heading) < 0.01))
                {
                    next_local_pt_.heading = GeneralFunctions::wrapTo2Pif(next_local_pt_.heading + controller_sampling_time_ * ang_vel);
                }

                next_local_pt_.x = next_local_pt_.x + cos(steeringAngle) * controller_sampling_time_ * lin_vel;
                next_local_pt_.y = next_local_pt_.y + sin(steeringAngle) * controller_sampling_time_ * lin_vel;
                //std::cout << "Trajectory Pose:\t[" << next_local_pt_.x << ", " << next_local_pt_.y << ", " << next_local_pt_.heading << "]" << std::endl;
                next_local_pt_.x_dot = cos(steeringAngle) * lin_vel; // cos(steeringAngle) * 0.1;
                next_local_pt_.y_dot = sin(steeringAngle) * lin_vel; // sin(steeringAngle) * 0.1;
                next_local_pt_.velocity_mps = lin_vel;
                next_local_pt_.acceleration_mps2 = 0.0;
                next_local_pt_.heading_acc_radps2 = 0.0;
                local_traj_msg_.points.push_back(next_local_pt_);
            }

            local_traj_pub_.publish(local_traj_msg_);

            // for the heading publish (doesnt work)
            //next_local_pt_.heading = steeringAngle;

            // ROS_INFO("Steering angle: %f", steeringAngle);

            // Generate a point that is 1cm away from the current position in the
            // direction of the target point and append to local_traj_msg_
            // double x_diff = targetPos.x() - x_pos_;
            // double y_diff = targetPos.y() - y_pos_;
            // double dist = sqrt(x_diff * x_diff + y_diff * y_diff);
            // dist = dist < 2.0 ? 2.0 : dist;
            /*
            steeringAngle = 0.5*(next_local_pt_.heading + steeringAngle);

            next_local_pt_.x = cos(steeringAngle) * 0.01 + x_pos_;
            next_local_pt_.y = sin(steeringAngle) * 0.01 + y_pos_;
            next_local_pt_.heading = steeringAngle;
            next_local_pt_.x_dot = cos(steeringAngle) * 0.1;
            next_local_pt_.y_dot = sin(steeringAngle) * 0.1;

            local_traj_msg_.points.push_back(next_local_pt_);
            */
        }
        else {
            ROS_ERROR("Steering angle out of bounds! Angle is %f", steeringAngle);
        }

        // publish path
        unsigned int number_of_points = 70;
        double step_size = 0.05;
        heading_path_msg_.poses.clear();
        heading_path_msg_.poses.resize(number_of_points);
        heading_path_msg_.header.seq += 1;
        heading_path_msg_.header.stamp = ros::Time::now();
        geometry_msgs::PoseStamped pose;
        pose.header.frame_id = "odom";
        pose.pose.position.x = x_pos_;
        pose.pose.position.y = y_pos_;
        pose.pose.position.z = 0.0;
        tf2::Quaternion quat_tf;
        quat_tf.setRPY(0.0, 0.0, heading_angle_);
        pose.pose.orientation = tf2::toMsg(quat_tf);
        heading_path_msg_.poses.at(0) = pose;
        for (unsigned int ii = 1; ii < number_of_points; ++ii)
        {
            pose.pose.position.x += step_size * std::cos(heading_angle_);
            pose.pose.position.y += step_size * std::sin(heading_angle_);
            heading_path_msg_.poses.at(ii) = pose;
        }
        heading_path_pub_.publish(heading_path_msg_);

        if (!(heading_vector_pub_.getNumSubscribers() < 1))
        {
            heading_vector_msg_.pose.position.x = x_pos_;
            heading_vector_msg_.pose.position.y = y_pos_;
            heading_vector_msg_.pose.position.z = 0.0;
            heading_vector_msg_.pose.orientation = tf2::toMsg(quat_tf);
            heading_vector_msg_.header.frame_id = "odom";
            heading_vector_pub_.publish(heading_vector_msg_);
        }
    }


public:
    ObstacleAvoidance(ros::NodeHandle nh, ros::NodeHandle p_nh, double alpha, double threshold, double param_a, double param_b,
                      int s_max, double d_max, double controller_sampling_time, double sampling_time)
    {
        // Set up node handles, publishers, subscribers
        nh_ = nh;
        private_nh_ = p_nh;

        local_traj_pub_ = private_nh_.advertise<sdv_msgs::Trajectory>("trajectory", 1);
        polar_hist_pub_ = private_nh_.advertise<nav_msgs::OccupancyGrid>("polar_histogram", 1);
        heading_vector_pub_ = private_nh_.advertise<geometry_msgs::PoseStamped>("heading_vfh", 1);
        heading_path_pub_ = private_nh_.advertise<nav_msgs::Path>("heading_path", 1);

        ROS_WARN("COLLISION_AVOIDANCE_WARN: odom and encoder_odom problem.");
        //odom_sub_ = nh_.subscribe("/encoder_odom", 4, &ObstacleAvoidance::odomCallback, this);
        odom_sub_ = nh_.subscribe("/odometry/filtered", 4, &ObstacleAvoidance::odomCallback, this);

        costmap_sub_ = nh_.subscribe("/pc_to_local_map", 2, &ObstacleAvoidance::costmapCallback, this);
        //goal_sub_ = nh_.subscribe("/move_base_simple/goal", 1, &ObstacleAvoidance::goalCallback, this);
        goal_sub_ = nh_.subscribe("/task_manager/vfh_pp_goal", 1, &ObstacleAvoidance::goalCallback, this);
        ROS_WARN("Subscriber adjust in collision avoidance function");
        // VFH parameters
        alpha_ = alpha  * PI / 180.0; // store alpa_ in radians for use in sin/cos functions
        threshold_ = threshold;
        param_a_ = param_a;
        param_b_ = param_b;
        s_max_ = s_max;
        d_max_ = d_max;
        controller_sampling_time_ = controller_sampling_time;

        unsigned long sectors = static_cast<unsigned long>(2 * PI / alpha_);
        densities_ = std::vector<double>(sectors);

        // Initialize messages
        histogram_grid_msg_.info.resolution = HIST_RESOLUTION;
        histogram_grid_msg_.info.width = HIST_WIDTH;
        histogram_grid_msg_.info.height = HIST_HEIGHT;

        histogram_grid_msg_.info.origin.position.x = - static_cast<double>(HIST_RESOLUTION) * HIST_WIDTH / 2;
        histogram_grid_msg_.info.origin.position.y = - static_cast<double>(HIST_RESOLUTION) * HIST_HEIGHT / 2;

        histogram_grid_msg_.header.frame_id = "odom";

        histogram_grid_msg_.data.resize(HIST_WIDTH * HIST_HEIGHT);


        //local_traj_msg_.points.reserve(10000);


        next_local_pt_.x = 0.0;
        next_local_pt_.y = 0.0;
        next_local_pt_.heading = 0.0;
        next_local_pt_.x_dot = 0.0; // cos(steeringAngle) * 0.1;
        next_local_pt_.y_dot = 0.0; // sin(steeringAngle) * 0.1;
        next_local_pt_.heading_rate_radps = 0.0;
        next_local_pt_.velocity_mps = 0.0;
        next_local_pt_.acceleration_mps2 = 0.0;
        next_local_pt_.heading_acc_radps2 = 0.0;

        heading_path_msg_.header.frame_id = "odom";
        heading_path_msg_.header.seq = 0;
        heading_path_msg_.header.stamp = ros::Time::now();

        // Initialize timer
        sampling_time_ = sampling_time;
        periodic_timer_ = private_nh_.createTimer(ros::Duration(sampling_time_), &ObstacleAvoidance::run, this);
    }


    void odomCallback(const nav_msgs::OdometryConstPtr odom_msg)
    {
        std::lock_guard<std::mutex> lock(odom_mutex_);
        x_pos_ = odom_msg->pose.pose.position.x;
        y_pos_ = odom_msg->pose.pose.position.y;
        double quatx = odom_msg->pose.pose.orientation.x;
        double quaty = odom_msg->pose.pose.orientation.y;
        double quatz = odom_msg->pose.pose.orientation.z;
        double quatw = odom_msg->pose.pose.orientation.w;
        tf::Quaternion q(quatx, quaty, quatz, quatw);
        tf::Matrix3x3 m(q);
        m.getRPY(roll_, pitch_, yaw_);
        heading_ = static_cast<float>(yaw_);
        angular_vel_ = odom_msg->twist.twist.angular.z;
        // ROS_INFO("Odom received. x: %f, y: %f", x_pos_, y_pos_);
    }


    void costmapCallback(const nav_msgs::OccupancyGrid::ConstPtr &costmap)
    {
        std::lock_guard<std::mutex> lock(costmap_mutex_);
        // ROS_INFO("Costmap callback");
        updateHistogram(costmap);
        updateHistogramMsg();
        // ROS_INFO("Costmap received.");
    }

    void slopemapCallback(const nav_msgs::OccupancyGrid::ConstPtr &slopemap)
    {
        //std::lock_guard<std::mutex> lock(costmap_mutex_);
    }

    void goalCallback(const geometry_msgs::PoseStamped &goal)
    {
        std::lock_guard<std::mutex> lock(goal_mutex_);
        goal_ << goal.pose.position.x, goal.pose.position.y;
    }

    void run(const ros::TimerEvent &event)
    {
        std::lock_guard<std::mutex> lock_traj(trajectory_mutex_);
        std::lock_guard<std::mutex> lock_odom(odom_mutex_);
        std::lock_guard<std::mutex> lock_costmap(costmap_mutex_);
        std::lock_guard<std::mutex> lock(goal_mutex_);

        // auto start_time = std::chrono::system_clock::now();

        createLocalTrajectory();

        // auto end_time = std::chrono::system_clock::now();
        // std::chrono::duration<double> elapsed_time = end_time - start_time;
        // std::cout << "Elapsed time: " << 1000*elapsed_time.count() << " [msec].\n" << std::endl;
    }


    void generateCSV()
    {
        std::ofstream export_data;
        export_data.open("local_traj.csv", std::ios::out | std::ios::trunc);
        export_data << "x, y, heading, x_dot, y_dot, heading_rate_radps, "
                       "velocity_mps, acceleration_mps2, heading_acc_radps2" << std::endl;
        for (size_t i = 0; i < local_traj_msg_.points.size(); ++i)
        {
            auto pt = local_traj_msg_.points[i];
            export_data << pt.x << ", " << pt.y << ", " << pt.heading << ", "
                        << pt.x_dot << ", " << pt.y_dot << ", " << pt.heading_rate_radps <<", "
                        << pt.velocity_mps << ", " << pt.acceleration_mps2 << ", "
                        << pt.heading_acc_radps2 << std::endl;
        }
        export_data.close();
    }
};


int main(int argc, char *argv[])
{
    ros::init(argc, argv, "obstacle_avoidance");
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");

    ros::Duration(2).sleep();

    // get necessary parameters from rosserver
    double controller_sampling_time;
    nh.param("controller_sampling_time", controller_sampling_time, 0.05);
    int s_max;
    nh.param("vfh_s_max", s_max, 40);
    double alpha;
    nh.param("vfh_alpha", alpha, 2.5);
    double d_max;
    nh.param("vfh_max_distance", d_max, 5.0);
    double a_param, b_param;
    nh.param("vfh_a", a_param, 10.0);
    // a-bd_max = 0
    b_param = a_param / d_max;
    double threshold;
    nh.param("vfh_threshold", threshold, 0.01);
    double sampling_time = 0.1;

    ObstacleAvoidance collisionAvoidance(nh, private_nh, 
                                         alpha, threshold, a_param, b_param,
                                         s_max, d_max, controller_sampling_time,
                                         sampling_time);

    ros::AsyncSpinner s(6);
    s.start();
    ros::waitForShutdown();

    //collisionAvoidance.generateCSV();

    return 0;
}


// int main(int argc, char *argv[])
// {
//     ros::init(argc, argv, "obstacle_avoidance");
//     ros::NodeHandle nh;

//     ros::Duration(2).sleep();

//     PolarHistogram polar_histogram(5.0, 10.0, 5.0, 1.0);
//     ros::Subscriber sub_costmap = nh.subscribe("/pc_to_local_map", 10, &PolarHistogram::updateHistogram, &polar_histogram);

//     ros::Publisher pub_histogram = nh.advertise<nav_msgs::OccupancyGrid>("/polar_histogram", 10);

//     // velodyne sim has 10hz
//     ros::Rate loop_rate(5);
//     std::clock_t start;
//     double duration;

//     while (ros::ok())
//     {
//         ros::spinOnce();

//         start = std::clock();
//         pub_histogram.publish(polar_histogram.toOccupancyGrid(0.05, 200, 200));
//         duration = (std::clock() - start) / double(CLOCKS_PER_SEC);

//         ROS_INFO("Running time: %f", duration);

//         loop_rate.sleep();
//     }

//     return 0;
// }
