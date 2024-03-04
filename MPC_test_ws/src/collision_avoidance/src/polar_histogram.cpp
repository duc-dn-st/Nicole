// ===============================================================================
// deprecated
// ===============================================================================

/*
 * ===============================================================================
 * polar_histogram.cpp
 * Author: Weidmann Daniel
 * Date: 20.01.2020
 * Email: weidmann.da@gmail.com
 * -------------------------------------------------------------------------------
 * Description:
 * This file implements a class to represent a polar histogram for the vector field
 * histogram algorithm.
 *
 * Subscribes to:
 *
 * Publishes:
 *
 * ===============================================================================
 */

#include <cmath>
#include <algorithm>

#include <boost/math/constants/constants.hpp>

#include "../include/collision_avoidance/polar_histogram.h"


constexpr double PI = boost::math::constants::pi<double>();

// public methods

PolarHistogram::PolarHistogram(double alpha, double threshold, double param_a, double param_b)
    : m_alpha{alpha * PI / 180.0},
      m_threshold{threshold}, m_param_a{param_a}, m_param_b{param_b}
{
    unsigned long sectors = static_cast<unsigned long>(2 * PI / m_alpha);

    this->m_densities = std::vector<double>(sectors);
}



/*!
 * Update the polar histogram from robot pose and costmap.
 * \brief PolarHistogram::updateHistogram
 * \param pose
 * \param costmap
 */
void PolarHistogram::updateHistogram(const nav_msgs::OccupancyGrid::ConstPtr &costmap)
{
    if (costmap != nullptr)
    {
        // clear the old histogram
        for (int ii = 0; ii < m_densities.size(); ++ii)
        {
            m_densities[ii] = 0;
        }

        // Position of robot inside the map
        int X = costmap->info.width / 2;
        int Y = costmap->info.height / 2;

        // maximum distance considered (only consider obstacles in a circle around robot)
        double max_dist = costmap->info.width * costmap->info.resolution / 2;

        // calculate obstacle vector for every cell in the costmap
        // use this obstacle vector to calculate obstacle density for every sectors
        for (int i = 0; i < costmap->info.width; ++i)
        {
            for (int j = 0; j < costmap->info.height; ++j)
            {
                double x_pos = static_cast<double>(i - X);
                double y_pos = static_cast<double>(j - Y);
                // angular position of this cell
                double beta = atan2( y_pos, x_pos );

                // magnitude of this cell
                double dist = sqrt( x_pos * x_pos + y_pos * y_pos ) * costmap->info.resolution;    // Euclidian distance from this cell to robot
                if (dist > max_dist) {
                    continue;     // this cell is not considered in building the histogram
                }

                double magn = 0.0;
                if ( costmap->data[i + j * costmap->info.width] >= 90.0 && costmap->data[i + j * costmap->info.width] <= 110.0 )
                {
                    magn = 1.0;
                }
                magn = magn * magn;
                magn *= m_param_a - m_param_b * dist;

                int sector = floor( (beta + PI) / m_alpha );
                m_densities[sector] += magn;
            }
        }

        // smooth the densities
        int l = 5;
        std::vector<double> smoothed_densities = std::vector<double>(m_densities.size());
        for (int i = 0; i < m_densities.size(); ++i)
        {
            double smoothed_density = 0;
            for (int j = -l+1; j < l; ++j) {
                int index = (i + j + m_densities.size()) %  m_densities.size();
                smoothed_density += abs(l - j) * m_densities[index];
            }
            smoothed_densities[i] = smoothed_density / (2*(double)l + 1.0);
        }
        m_densities = smoothed_densities;
    }
}


/*!
 * Get a representation of the polar histogram as OccupancyGrid
 * \brief PolarHistogram::toOccupancyGrid
 * \param resolution the resolution of the map
 * \param width, height the size of the map
 * \return an OccupancyGrid representation of the polar histogram
 */
nav_msgs::OccupancyGrid PolarHistogram::toOccupancyGrid(float resolution, int width, int height)
{
    nav_msgs::OccupancyGrid histogram_grid_msg;

    histogram_grid_msg.info.resolution = resolution;
    histogram_grid_msg.info.width = width;
    histogram_grid_msg.info.height = height;

    histogram_grid_msg.info.origin.position.x = - (double)resolution * width / 2;
    histogram_grid_msg.info.origin.position.y = - (double)resolution * height / 2;

    histogram_grid_msg.header.frame_id = "odom";

    histogram_grid_msg.data.resize(width * height);

    int X = width / 2;
    int Y = height / 2;

    double max_density = *max_element(m_densities.begin(), m_densities.end());
    // ROS_WARN("Max element is %f", max_density);
    // ROS_WARN("Drawing polar histogram");

    bool draw_histogram = true;
    if (draw_histogram) {
        // Fill the map, each cell belongs to one of the sectors of the polar histogram
        for (int i = 0; i < width; ++i)
        {
            for (int j = 0; j < height; ++j)
            {
                // Determine sector of this cell
                double beta = atan2( static_cast<double>(j - Y), static_cast<double>(i - X) );
                int sector = (int)floor( (beta + PI) / m_alpha ) % m_densities.size();
                if (sector < 0 or sector > 71) ROS_ERROR("Illegal sector!");


                double dens = m_densities[sector] * 100.0;
                if (max_density != 0.0) dens = dens / max_density;
                if (dens < 1.0) {
                    dens = 0.0;
                } else if (dens < 20.0) {
                    dens = 15.0;
                } else if (dens < 40.0) {
                    dens = 30.0;
                } else if (dens < 60.0) {
                    dens = 45.0;
                } else if (dens < 80.0) {
                    dens = 60.0;
                } else {
                    dens = 75.0;
                }

                histogram_grid_msg.data[i + j * width] = dens;
                // ROS_INFO("%f", dens);
            }
        }
    } else {
        ROS_INFO("Besenham");
        // Plot a straight line with value 100 for the target direction
        Eigen::Vector2d current(0, 0);
        Eigen::Vector2d target(4, 1);
        double steeringAngle = calcDirection(current, target);
        ROS_WARN("%f", steeringAngle);

        // Bresenham's line algorithm (https://en.wikipedia.org/wiki/Bresenham%27s_line_algorithm)
        double radius = histogram_grid_msg.info.width / 2;
        int x0 = histogram_grid_msg.info.width / 2;
        int y0 = histogram_grid_msg.info.height / 2;
        int x1 = cos(steeringAngle) * radius + x0;
        int y1 = sin(steeringAngle) * radius + y0;

        int dx = abs(x1 - x0);
        int sx = x0 < x1? 1 : -1;
        int dy = -abs(y1 - y0);
        int sy = y0 < y1? 1 : -1;
        int err = dx + dy;
        while (true) {
            histogram_grid_msg.data[x0 + y0 * width] = 100.0;   // "plot"
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

        ROS_INFO("Besenham done");
    }

    return histogram_grid_msg;
}


/*!
 * Calculate the steering angle from the polar histogram
 * \brief PolarHistogram::calcDirection
 * \param currentPos the coordinates of the robot
 * \param laPoint the coordinates of the target
 * \return the steering angle to avoid obstacles
 */
double PolarHistogram::calcDirection(Eigen::Vector2d currentPos, Eigen::Vector2d laPoint) {
    // get sector in which the target lies
    int targetSector = laSector(currentPos, laPoint);

    // find valleys, possibly close to the target sector
    std::vector<int> valleys = findValleys(targetSector);

    int nearest = 0;
    for (int ii; ii < valleys.size(); ++ii) {
        int v = valleys[ii];
        if (abs(v - targetSector) < abs(nearest - targetSector)) {
            nearest = v;
        }
    }

    double steering;
    if (nearest == targetSector) {      // if target sector is the nearest, directly steer towards the target angle
        steering = atan2(laPoint[1] - currentPos[1], laPoint[0] - currentPos[0]);
    } else {        // else steer towards the center of the nearest valley
        steering = static_cast<int>(nearest) * m_alpha + 0.5*m_alpha + PI;
    }
    steering = (steering > 2*PI)? (steering - 2*PI) : steering;
    return steering;
}



// private helper funtions

/*!
 * Calculate the sector that matches the target direction
 * \brief PolarHistogram::laSector
 * \param currentPos the coordinates of the robot
 * \param laPoint the coordinates of the target
 * \return index of the sector that matches the target direction
 */
int PolarHistogram::laSector(Eigen::Vector2d currentPos, Eigen::Vector2d laPoint) {
    double laAngle = atan2(laPoint[1] - currentPos[1], laPoint[0] - currentPos[0]);
    return (int)floor( (laAngle + PI) / m_alpha ) % m_densities.size();
}


/*!
 * Calculate all candidate valleys for the robot to pass through
 * \brief PolarHistogram::findValleys
 * \param targetSector the sector of the target position
 * \return a vector of all candidate valleys
 */
std::vector<int> PolarHistogram::findValleys(int targetSector) {
    // (copied from Matthew's code)
    int k = 0, j = 0, kr, kl;
    int smax = 2; //width of the valley
    std::vector<int> candidates;
    while (k <= m_densities.size())
    {
        if (m_densities[k] < m_threshold)
        {
                kr = k; // right border of a valley
                        while (k <= m_densities.size() && m_densities[k]< m_threshold)
                        {
                                kl = k; // left border of a valley
                                k = k + 1;
                        }
                // calculate candidate directions :
                if (kl - kr > smax) // wide valley
                {
                        candidates.push_back(static_cast<int>(kl - smax / 2)); // towards the left side
                        candidates.push_back(static_cast<int>(kr + smax / 2)); // towards the right side
                        j = j + 2;
                        if (targetSector >= kr && targetSector <= kl)
                        {
                                candidates.push_back(targetSector); // straight at look ahead
                                j = j + 1;
                        }
                }
                else if(kl - kr > smax / 5) // narrow valley
                {
                        candidates.push_back(static_cast<int>((kr + kl) / 2));
                        j = j + 1;
                }
        }
        else
        {
                k = k + 1;
        }
    }

    return candidates;
}
