// ===============================================================================
// deprecated
// ===============================================================================

/*
 * ===============================================================================
 * polar_histogram.h
 * Author: Weidmann Daniel
 * Date: 20.01.2020
 * Email: weidmann.da@gmail.com
 * -------------------------------------------------------------------------------
 * Description:
 * This file defines a class to represent a polar histogram for the vector field
 * histogram algorithm.
 *
 * Subscribes to:
 *
 * Publishes:
 *
 * ===============================================================================
 */

#ifndef POLARHISTOGRAM_H
#define POLARHISTOGRAM_H

#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/Pose.h>
#include <eigen3/Eigen/Dense>

#include <vector>

#include "sdv_msgs/Trajectory.h"
#include "sdv_msgs/TrajectoryPoint.h"


class PolarHistogram {
public:
    /// Constructor
    PolarHistogram(double alpha, double threshold, double param_a, double param_b);

    // Update the polar histogram using robot pose and costmap
    void updateHistogram(const nav_msgs::OccupancyGrid::ConstPtr &costmap);

    // Returns a mapping of the polar histogram to an OccupancyGrid
    nav_msgs::OccupancyGrid toOccupancyGrid(float resolution, int width, int height);

    // Calculates the steering angle around obstacle respecting target point
    double calcDirection(Eigen::Vector2d currentPos, Eigen::Vector2d laPoint);

    // Return a Trajectory to avoid a detected obstacle
    sdv_msgs::Trajectory obstacleAvoidanceTrajectory();

private:
    double m_alpha;                     // angular resolution (in rad)
    double m_threshold;                 // threshold for sector to be passable
    double m_param_a, m_param_b;        // parameters for the calculation of obstacle vector magnitude
    std::vector<double> m_densities;    // obstacle density per sector

    // private helper methods
    inline int laSector(Eigen::Vector2d currentPos, Eigen::Vector2d laPoint);
    std::vector<int> findValleys(int targetSector);
};


#endif // POLARHISTOGRAM_H
