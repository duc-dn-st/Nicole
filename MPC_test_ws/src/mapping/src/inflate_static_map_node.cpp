/*
 * ===============================================================================
 * inflate_static_map_node.cpp
 * Author: Schaefle Tobias
 * Date: 26.02.2021
 * Email: tobias.schaefle@gmail.com
 * -------------------------------------------------------------------------------
 * Description:
 * Inflates the static map with a given radius.
 * ===============================================================================
 */

#define INFLATE_RADIUS 1

#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>

#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Core>

#include <math.h>

class InflateStaticMap
{
public:
    InflateStaticMap(ros::NodeHandle nh, ros::NodeHandle private_nh)
    {
        nh_ = nh;
        private_nh_ = private_nh;

        getStaticMap();

        map_pub_ = private_nh_.advertise<nav_msgs::OccupancyGrid>("inflated_map", 1);
    }

    void inflateMap()
    {
        int cell_x, cell_y;
        cell_x = 0;
        cell_y = 0;
        int counter = -1;
        for (unsigned int ii = 0; ii < static_map_->info.height; ++ii)
        {
            for (unsigned int jj = 0; jj < static_map_->info.width; ++jj)
            {
                unsigned int index = jj + ii * static_map_->info.width;
                if (static_map_->data[index] > 0)
                {
                    double x, y;
                    x = jj * static_map_->info.resolution;
                    y = ii * static_map_->info.resolution;
                    inflated_map_.data[index] = 100;
                    // inflate
                    float radius = inflated_map_.info.resolution;
                    while (radius <= INFLATE_RADIUS)
                    {
                        midpointCircleAlgorithm(x, y, radius, 50);
                        radius += static_map_->info.resolution;
                    }
                }
            }

        }
        map_pub_.publish(inflated_map_);
    }

private:
    void getStaticMap()
    {
        static_map_ = ros::topic::waitForMessage<nav_msgs::OccupancyGrid>("map", nh_, ros::Duration(20.0));
        if (static_map_ == nullptr)
        {
            ROS_ERROR("No static map received.");
        }
        else
        {
            inflated_map_.data.resize(static_map_->data.size());
            inflated_map_.info = static_map_->info;
            inflated_map_.header = static_map_->header;
        }

    }

    void updateAllOctants(float x, float y, float *obstacle_x, float *obstacle_y, signed char inflated_value)
    {
        float x_obst, y_obst;

        // 1 octant (x, y)
        x_obst = x + *obstacle_x;
        y_obst = y + *obstacle_y;
        updateInflatedValues(x_obst, y_obst, inflated_value);

        // 2 octant (y, x)
        x_obst = y + *obstacle_x;
        y_obst = x + *obstacle_y;
        updateInflatedValues(x_obst, y_obst, inflated_value);

        // 3 octant (-y, x)
        x_obst = -y + *obstacle_x;
        y_obst = x + *obstacle_y;
        updateInflatedValues(x_obst, y_obst, inflated_value);

        // 4 octant (-x, y)
        x_obst = -x + *obstacle_x;
        y_obst = y + *obstacle_y;
        updateInflatedValues(x_obst, y_obst, inflated_value);

        // 5 octant (-x, -y)
        x_obst = -x + *obstacle_x;
        y_obst = -y + *obstacle_y;
        updateInflatedValues(x_obst, y_obst, inflated_value);

        // 6 octant (-y, -x)
        x_obst = -y + *obstacle_x;
        y_obst = -x + *obstacle_y;
        updateInflatedValues(x_obst, y_obst, inflated_value);

        // 7 octant (y, -x)
        x_obst = y + *obstacle_x;
        y_obst = -x + *obstacle_y;
        updateInflatedValues(x_obst, y_obst, inflated_value);

        // 8 octant (x, -y)
        x_obst = x + *obstacle_x;
        y_obst = -y + *obstacle_y;
        updateInflatedValues(x_obst, y_obst, inflated_value);
    }

    void midpointCircleAlgorithm(float obstacle_x, float obstacle_y, float radius, signed char inflate_cost)
    {
        // start position for midpoint(0, 0)
        float x = radius;
        float y = 0.0f;

        updateAllOctants(x, y, &obstacle_x, &obstacle_y, inflate_cost);

        //iteration in the first octant
        while (x > y)
        {
            // currently px = local_map_msg_.info.resolution
            // x+1^2 = x^2 - 2y - 1 (1px) = x^2 - 2y px - px^2
            x = sqrtf(x * x - 2 * y * inflated_map_.info.resolution - (inflated_map_.info.resolution * inflated_map_.info.resolution));
            // y+1^2 = (y + 1)^2 (1px) -> y+1 = y + 1px
            y = y + inflated_map_.info.resolution;

            updateAllOctants(x, y, &obstacle_x, &obstacle_y, inflate_cost);
        }
    }

    void updateInflatedValues(float x, float y, signed char inflate_value)
    {
        // get cell pos x and y
        int cell_x = int(round(double(x / inflated_map_.info.resolution)));
        int cell_y = int(round(double(y / inflated_map_.info.resolution)));

        // map index
        unsigned int index = static_cast<unsigned int>(cell_x) + inflated_map_.info.width * static_cast<unsigned int>(cell_y);
        if (inGrid(cell_x, cell_y))
        {
            // check if cell is already assigned with a higher value
            if (inflated_map_.data[index] < inflate_value)
                inflated_map_.data[index] = inflate_value;
        }
    }

    bool inGrid(signed int cell_x, signed int cell_y)
    {
        if ((cell_x < 0 || cell_x >= static_cast<signed int>(inflated_map_.info.width))
                || (cell_y < 0 || cell_y >= static_cast<signed int>(inflated_map_.info.height)))
        {
            return false;
        }
        else {return true;}
    }

    nav_msgs::OccupancyGrid inflated_map_;
    boost::shared_ptr<nav_msgs::OccupancyGrid const> static_map_;

    ros::NodeHandle nh_, private_nh_;

    ros::Publisher map_pub_;
};

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "inflate_static_map_node");
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");

    InflateStaticMap inflateMap(nh, private_nh);

    ros::Rate loop_rate(1);

    while (ros::ok())
    {
        ros::spinOnce();

        inflateMap.inflateMap();

        loop_rate.sleep();
    }

    return 0;
}
