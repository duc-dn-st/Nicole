/*
 * ===============================================================================
 * local_map.cpp
 * Author: Schaefle Tobias
 * -------------------------------------------------------------------------------
 * Description:
 * This .cpp file generates a local map for various path planners
 * the grid will be generated by the laser scans
 *
 * Subscribes to:
 * - lidar/scan
 *
 * Publishes:
 * - Data type: nav_msgs/OccupancyGrid
 * - Address: local_map
 * ===============================================================================
 */
#include "ros/ros.h"
#include "nav_msgs/OccupancyGrid.h"
#include "nav_msgs/MapMetaData.h"
#include "nav_msgs/Odometry.h"
#include "sensor_msgs/LaserScan.h"
#include "tf/transform_listener.h"
#include "math.h"

class LocalMap
{
private:
    ros::NodeHandle nh_;
    ros::Publisher local_map_;
    ros::Publisher map_metadata_;

    tf::StampedTransform transform_odom_lidar_;
    tf::StampedTransform transform_odom_base_;
    // published format
    nav_msgs::OccupancyGrid local_map_msg_;
    nav_msgs::MapMetaData map_metadata_msg_;

public:
    LocalMap(ros::NodeHandle nh, tf::StampedTransform transform_odom_lidar, tf::StampedTransform transform_odom_base,
                        float cell_size, unsigned int width, unsigned int height)
    {
	this->nh_ = nh;
	this->local_map_ = nh_.advertise<nav_msgs::OccupancyGrid>("local_map", 10);
	this->map_metadata_ = nh_.advertise<nav_msgs::MapMetaData>("local_map_metadata", 10);
	this->transform_odom_lidar_ = transform_odom_lidar;
	this->transform_odom_base_ = transform_odom_base;

	this->map_metadata_msg_.resolution = cell_size;
	this->map_metadata_msg_.width = width;
	this->map_metadata_msg_.height = height;
	// first get it in meter (cellsize times width) then it starts by minus half of that [bottom left corner]
	this->map_metadata_msg_.origin.position.x = -static_cast<double>(cell_size) * width / 2;
	this->map_metadata_msg_.origin.position.y = -static_cast<double>(cell_size) * height / 2;

	this->local_map_msg_.info = map_metadata_msg_;

	this->local_map_msg_.data.resize(width * height);

	this->local_map_msg_.header.frame_id = "odom";

    }

    /*
    // converts the fine grained occupancy grid of SLAM into a coarse grid (for TASP and HA* Planning)
    void getSLAMMapCallback(const nav_msgs::OccupancyGrid::ConstPtr &slam_msg)
    {

    }
    */

    // can be used for local map generation at each sample instant
    void getLaserScanCallback(const sensor_msgs::LaserScan::ConstPtr &laser_msg)
    {
	// index in grid array
	unsigned int index;

	signed int cell_x;
	signed int cell_y;

	tf::Vector3 lidar_point;
	tf::Vector3 odom_point;
	tf::Vector3 rel_cell_num;
	tf::Vector3 rel_point;

	tf::Vector3 base_to_odom;
	base_to_odom.setX(transform_odom_lidar_.getOrigin().getX());
	base_to_odom.setY(transform_odom_lidar_.getOrigin().getY());

	// update the position of the map based on the current robots position
	map_metadata_msg_.origin.position.x = transform_odom_base_.getOrigin().getX() - static_cast<double>(map_metadata_msg_.resolution) * map_metadata_msg_.width / 2;
	map_metadata_msg_.origin.position.y = transform_odom_base_.getOrigin().getY() - static_cast<double>(map_metadata_msg_.resolution) * map_metadata_msg_.height / 2;
	//map_metadata_msg_.origin.orientation = odom_msg_.pose.pose.orientation;
	local_map_msg_.info = map_metadata_msg_;

	// maybe map should be newly initialized at each round and TASP will work with the newest local map
	for (unsigned long jj = 0; jj < local_map_msg_.info.width * local_map_msg_.info.height; jj++)
	{
	    local_map_msg_.data[jj] = 0;
	}

	for (unsigned long ii = 0; ii < laser_msg->ranges.size(); ii++)
	{
	    if (!isinf(static_cast<double>(laser_msg->ranges[ii])))
	    {
		// obstacle position in lidar frame
		lidar_point.setX(static_cast<double>(laser_msg->ranges[ii]) * cos(static_cast<double>(laser_msg->angle_increment) * ii));
		lidar_point.setY(static_cast<double>(laser_msg->ranges[ii]) * sin(static_cast<double>(laser_msg->angle_increment) * ii));

		//ROS_INFO("Angle:/t%f/tPos:/t%f, %f", static_cast<double>(laser_msg->angle_increment) * ii, lidar_x, lidar_y);

		// obstacle position in odom frame
		odom_point = transform_odom_lidar_ * lidar_point;
		//ROS_INFO("x odom:\t%f\tx lidar:\t%f", odom_point.getX(), lidar_point.getX());

		// get relativ coordinates with respect to the base_link
		rel_point = - base_to_odom + odom_point;

		rel_cell_num = rel_point / static_cast<double>(local_map_msg_.info.resolution);
		//ROS_INFO("cell x:\t%f\tcell y:\t%f", round(cell_num.getX()) + width_ / 2, round(cell_num.getY()) + height_ / 2);

		// local_map_msg_.info.width / 2 + cell_num.getX() returns cell row
		cell_x = static_cast<signed int>(round(rel_cell_num.getX())) + static_cast<signed int>(local_map_msg_.info.width / 2);
		cell_y = static_cast<signed int>(round(rel_cell_num.getY())) + static_cast<signed int>(local_map_msg_.info.height / 2);

		if (!inGrid(cell_x, cell_y))
		{
		    ROS_WARN("Dimensions of the map are not big enough!\nCell X:\t%d\tCell Y:\t%d", cell_x, cell_y);
		}
		else
		{
		    // get index with the formula = x + info.width * y
		    index = static_cast<unsigned int>(cell_x) + local_map_msg_.info.width * static_cast<unsigned int>(cell_y);
		    local_map_msg_.data[index] = 100;
		    updateNeighbours(cell_x, cell_y);
		}
	    }
	}
	//local_map_msg_.data[0] = -1;
	//local_map_msg_.data[1] = -1;
	map_metadata_.publish(map_metadata_msg_);
	local_map_.publish(local_map_msg_);
    }

    void updateNeighbours(signed int cell_x, signed int cell_y)
    {
	// iterate array for neighbours
	int next[5] = {-2, -1, 0, 1, 2};
	unsigned int index;

	for (int ii = 0; ii < 5; ii++)
	{
	    for (int jj = 0; jj < 5; jj++)
	    {
		//ROS_INFO("Cell X:\t%d\tCell Y:\t%d", cell_x + next[ii], cell_y + next[jj]);
		if (inGrid(cell_x + next[ii], cell_y + next[jj]))
		{
		    index = static_cast<unsigned int>(cell_x + next[ii]) + local_map_msg_.info.width * static_cast<unsigned int>(cell_y + next[jj]);
		    if (local_map_msg_.data[index] < 50)
		    {
			local_map_msg_.data[index] = 50;
		    }
		}
	    }
	}

    }

    void setTransform(tf::StampedTransform transform_odom_lidar, tf::StampedTransform transform_odom_base)
    {
	this->transform_odom_lidar_ = transform_odom_lidar;
	this->transform_odom_base_ = transform_odom_base;
    }

    bool inGrid(signed int cell_x, signed int cell_y)
    {
	if ((cell_x < 0 || cell_x >= static_cast<signed int>(local_map_msg_.info.width))
	        || (cell_y < 0 || cell_y >= static_cast<signed int>(local_map_msg_.info.height)))
	{
	    return false;
	}
	else {return true;}
    }

};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "local_map");

    ros::NodeHandle nh;
    tf::TransformListener listener;
    tf::StampedTransform transform_odom_lidar;
    tf::StampedTransform transform_odom_base;
    try
    {
	ros::Duration(1.0).sleep();
	listener.lookupTransform("odom", "lidar", ros::Time(0), transform_odom_lidar);
	listener.lookupTransform("odom", "base_link", ros::Time(0), transform_odom_base);
    }
    catch (tf::TransformException &ex)
    {
	ROS_ERROR("%s", ex.what());
	ros::Duration(1.0).sleep();
    }

    // set cell parameters
    float cell_size = 0.5;
    unsigned int width = 36;
    unsigned height = 36;

    LocalMap local_map(nh, transform_odom_lidar, transform_odom_base, cell_size, width, height);
    ros::Subscriber subLaser = nh.subscribe("lidar/scan", 1000, &LocalMap::getLaserScanCallback, &local_map);

    ros::Rate loop_rate(5);

    while(ros::ok())
    {
	ros::spinOnce();

	try
	{
	    listener.lookupTransform("odom", "lidar", ros::Time(0), transform_odom_lidar);
	    listener.lookupTransform("odom", "base_link", ros::Time(0), transform_odom_base);
	}
	catch (tf::TransformException &ex)
	{
	    ROS_ERROR("%s", ex.what());
	    ros::Duration(1.0).sleep();
	}

	// update the odom transform in the class
	local_map.setTransform(transform_odom_lidar, transform_odom_base);

	loop_rate.sleep();
    }

    return 0;
}