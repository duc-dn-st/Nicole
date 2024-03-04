/*
 * ===============================================================================
 * pc_to_local_map_node.cpp
 * Author: Schaefle Tobias
 * Date: 25.05.2019
 * -------------------------------------------------------------------------------
 * Description:
 * This .cpp file generates a local map from the two velodyne sensor pointclouds
 *
 * Subscribes to:
 * - /velodyne_points_back (sensor_msgs/PointCloud2)
 * - /velodyne_points_front (sensor_msgs/PointCloud2)
 *
 * Publishes:
 * - Data type: nav_msgs/occupancy_grid
 * - Address: <__ROS ADDRESS__>
 * ===============================================================================
 */


// Include files from ROS libraries
#include <ros/ros.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/MapMetaData.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_listener.h>
#include <tf/tf.h>

#include <eigen3/Eigen/Core>
// Include files of self-made libraries


// Include files of other libraries
#include <math.h>
#include <ctime>

class PointCloudToMap
{

private:

    // ROS vars
    ros::NodeHandle nh_;
    ros::Publisher pub_local_map_;
    ros::Publisher map_metadata_;

    // Publisher vars
    nav_msgs::OccupancyGrid local_map_msg_;
    nav_msgs::MapMetaData map_metadata_msg_;

    // tf vars
    tf::TransformListener listener_;

    // Subscriber vars
    sensor_msgs::PointCloud2::ConstPtr velo_front_;
    sensor_msgs::PointCloud2::ConstPtr velo_back_;

    const double robot_x_size_ = 0.85;
    const double robot_y_size_ = 0.7;
    // Private methods
    /*!
     * \brief inGrid checks if the cell coordinates are in the grid of the map
     * \param cell_x x cell coordinate for the obstacle
     * \param cell_y y cell coordinate for the obstacle
     * \return boolean: true if in map otherwise false
     */
    bool inGrid(signed int cell_x, signed int cell_y)
    {
	if ((cell_x < 0 || cell_x >= static_cast<signed int>(local_map_msg_.info.width))
	        || (cell_y < 0 || cell_y >= static_cast<signed int>(local_map_msg_.info.height)))
	{
	    return false;
	}
	else {return true;}
    }

    /*!
     * \brief getTransformedCloudPoints
     * \param transform
     * \param point_array
     * \param x
     * \param y
     * \param z
     */
    void getTransformedCloudPoints(tf::StampedTransform &transform_base_velo, tf::StampedTransform &transform_odom_base, sensor_msgs::PointCloud2ConstIterator<float> point_array,
                                   Eigen::Vector3f &coordinates, bool &inRobot)
    {
        /*
        double roll, pitch, yaw;
        transform_odom_base.getBasis().getRPY(roll, pitch, yaw);
        float x_base = point_array[0] + float(transform_base_velo.getOrigin().x());
        float y_base = point_array[1] + float(transform_base_velo.getOrigin().y());
        float z_base = point_array[2] + float(transform_base_velo.getOrigin().z());

        // rotate coordinate
        x = float(cos(yaw)) * x_base - float(sin(yaw)) * y_base;
        y = float(sin(yaw)) * x_base + float(cos(yaw)) * y_base;
        z = z_base;
        */

        tf::Transform trans_odom_map;
        tf::poseMsgToTF(local_map_msg_.info.origin, trans_odom_map);
        geometry_msgs::Pose point;
        point.position.x = point_array[0];
        point.position.y = point_array[1];
        point.position.z = point_array[2];
        tf::Pose tf_pose;
        tf::poseMsgToTF(point, tf_pose);
        tf_pose = trans_odom_map.inverse() * transform_odom_base * transform_base_velo * tf_pose;
        coordinates << tf_pose.getOrigin().x(), tf_pose.getOrigin().y(), tf_pose.getOrigin().z();

        // check robot box
        tf::poseMsgToTF(point, tf_pose);
        tf_pose = transform_base_velo * tf_pose;
        inRobot = fabs(double(tf_pose.getOrigin().getX())) < robot_x_size_ && fabs(double(tf_pose.getOrigin().getY())) < robot_y_size_;

    }

    // function for rotating the map
    /*
    void getTransformedCloudPoints(tf::StampedTransform *transform, sensor_msgs::PointCloud2ConstIterator<float> point_array, float &x, float &y, float &z)
    {
	float x_base = point_array[0] + float(transform->getOrigin().x());
	float y_base = point_array[1] + float(transform->getOrigin().y());
	float z_base = point_array[2] + float(transform->getOrigin().z());
    }
    */

    /*!
     * \brief updateMapInfo
     * \param transform_odom_base
     */
    void updateMapInfo(tf::StampedTransform &transform_odom_base)
    {
	// run the following commented code to rotate the map with the robot orientation
	/*
	// update the position and orientation of the map based on the current robots position and ori
	double roll, pitch, yaw;
	transform_odom_base->getBasis().getRPY(roll, pitch, yaw);
	//ROS_INFO("%f", yaw);
	map_metadata_msg_.origin.position.x = transform_odom_base->getOrigin().getX()
		+ cos(yaw) * (-double(map_metadata_msg_.resolution) * map_metadata_msg_.width / 2)
		- sin(yaw) * (-double(map_metadata_msg_.resolution) * map_metadata_msg_.height / 2);
	map_metadata_msg_.origin.position.y = transform_odom_base->getOrigin().getY()
		+ sin(yaw) * (-double(map_metadata_msg_.resolution) * map_metadata_msg_.width / 2)
		+ cos(yaw) * (-double(map_metadata_msg_.resolution) * map_metadata_msg_.height / 2);
	map_metadata_msg_.origin.orientation.w = transform_odom_base->getRotation().getW();
	map_metadata_msg_.origin.orientation.x = transform_odom_base->getRotation().getX();
	map_metadata_msg_.origin.orientation.y = transform_odom_base->getRotation().getY();
	map_metadata_msg_.origin.orientation.z = transform_odom_base->getRotation().getZ();
	local_map_msg_.info = map_metadata_msg_;
	*/
	// the following code is for a non rotating map
    map_metadata_msg_.origin.position.x = transform_odom_base.getOrigin().getX() - double(map_metadata_msg_.resolution) * map_metadata_msg_.width / 2;
    map_metadata_msg_.origin.position.y = transform_odom_base.getOrigin().getY() - double(map_metadata_msg_.resolution) * map_metadata_msg_.height / 2;
	local_map_msg_.info = map_metadata_msg_;
    }

public:

    PointCloudToMap(ros::NodeHandle nh, float cell_size, unsigned int width, unsigned int height)
    {
	this->nh_ = nh;
	this->pub_local_map_ = nh_.advertise<nav_msgs::OccupancyGrid>("pc_to_local_map", 10);
	this->map_metadata_ = nh_.advertise<nav_msgs::MapMetaData>("map_metadata", 10);

	// metadata information
	this->map_metadata_msg_.resolution = cell_size;
	this->map_metadata_msg_.width = width;
	this->map_metadata_msg_.height = height;
	// first get it in meter (cellsize times width) then it starts by minus half of that [bottom left corner]
	this->map_metadata_msg_.origin.position.x = -double(cell_size) * width / 2;
	this->map_metadata_msg_.origin.position.y = -double(cell_size) * height / 2;
    this->map_metadata_msg_.origin.orientation.w = 1;

	// info of the local map
	this->local_map_msg_.info = map_metadata_msg_;
	this->local_map_msg_.data.resize(width * height);
	this->local_map_msg_.header.frame_id = "odom";

	// create nullptr for pointclouds
	velo_front_ = nullptr;
	velo_back_ = nullptr;
    }

    /*!
     * \brief getFrontVelodyneCallback
     * \param front_velo_msg
     */
    void getFrontVelodyneCallback(const sensor_msgs::PointCloud2::ConstPtr &front_velo_msg)
    {
	velo_front_ = front_velo_msg;
    }

    void getFrontVeloMap(tf::StampedTransform &transform_base_velo, tf::StampedTransform &transform_odom_base)
    {
	// if velo is a nullptr return with nothing
	if (velo_front_ == nullptr) {return;}
	// cell pos x and y
	int cell_x, cell_y;
	// map index
	unsigned int index;
	// boolean to remove points which appear on the robot
	bool inRobot;

	/*
	// make a transform object
	tf::StampedTransform transform_base_velo;
	tf::StampedTransform transform_odom_base;

	// query listener for specific transform;
	while (true)
	{
	    try
	    {
		listener_.lookupTransform("/base_link", "/velodyne_front", ros::Time(0), transform_base_velo);
		listener_.lookupTransform("/odom", "/base_link", ros::Time(0), transform_odom_base);
		break;
	    }
	    catch (tf::TransformException &ex)
	    {
		ROS_ERROR("%s", ex.what());
		ros::Duration(1.0).sleep();
	    }
	}

	updateMapInfo(&transform_odom_base);
	*/

	// iterate through pointcloud
	for (sensor_msgs::PointCloud2ConstIterator<float> iter_x(*velo_front_, "x"); iter_x != iter_x.end(); ++iter_x)
	{
	    //ROS_INFO("%f", transform.getOrigin().x());
	    //getTransformedCloudPoints(&transform_base_velo, iter_x, x, y, z);
        Eigen::Vector3f coordinates;
        getTransformedCloudPoints(transform_base_velo, transform_odom_base, iter_x, coordinates, inRobot);

        if (coordinates[2] > 0.005f && !inRobot)
	    {
		// get cell pos x and y
        cell_x = int(round(double(coordinates[0] / local_map_msg_.info.resolution)));
        cell_y = int(round(double(coordinates[1] / local_map_msg_.info.resolution)));

		// map index
		index = static_cast<unsigned int>(cell_x) + local_map_msg_.info.width * static_cast<unsigned int>(cell_y);
		if (inGrid(cell_x, cell_y))
		{
		    local_map_msg_.data[index] = 100;
		}
	    }
	}
    }

    /*!
     * \brief getBackVelodyneCallback
     * \param back_velo_msg
     */
    void getBackVelodyneCallback(const sensor_msgs::PointCloud2::ConstPtr &back_velo_msg)
    {
	velo_back_ = back_velo_msg;
    }

    void getBackVeloMap(tf::StampedTransform &transform_base_velo, tf::StampedTransform &transform_odom_base)
    {
	// if velo is a nullptr return with nothing
	if (velo_back_ == nullptr) {return;}
	// cell pos x and y
	int cell_x, cell_y;
	// map index
	unsigned int index;
	// boolean to remove points which appear on the robot
	bool inRobot;

	/*
	// make a transform object
	tf::StampedTransform transform_base_velo;
	tf::StampedTransform transform_odom_base;

	// query listener for specific transform;
	while (true)
	{
	    try
	    {
		listener_.lookupTransform("/base_link", "/velodyne_back", ros::Time(0), transform_base_velo);
		listener_.lookupTransform("/odom", "/base_link", ros::Time(0), transform_odom_base);
		break;
	    }
	    catch (tf::TransformException &ex)
	    {
		ROS_ERROR("%s", ex.what());
		ros::Duration(1.0).sleep();
	    }
	}


	updateMapInfo(&transform_odom_base);
	*/

	// iterate through pointcloud
	for (sensor_msgs::PointCloud2ConstIterator<float> iter_x(*velo_back_, "x"); iter_x != iter_x.end(); ++iter_x)
	{
	    //getTransformedCloudPoints(&transform_base_velo, iter_x, x, y, z);
        Eigen::Vector3f coordinates;
        getTransformedCloudPoints(transform_base_velo, transform_odom_base, iter_x, coordinates, inRobot);

        if (coordinates[2] > 0.005f && !inRobot)
	    {
		// get cell pos x and y
        cell_x = int(round(double(coordinates[0] / local_map_msg_.info.resolution)));
        cell_y = int(round(double(coordinates[1] / local_map_msg_.info.resolution)));
		// map index
		index = static_cast<unsigned int>(cell_x) + local_map_msg_.info.width * static_cast<unsigned int>(cell_y);
		if (inGrid(cell_x, cell_y))
		{
		    local_map_msg_.data[index] = 100;
		}
	    }
	}
    }

    /*!
     * \brief publishLocalMap
     */
    void publishLocalMap(tf::StampedTransform &odom_base, tf::StampedTransform &base_velo_front, tf::StampedTransform &base_velo_back)
    {
	// update map position
	updateMapInfo(odom_base);

	// get map from velodynes
	getFrontVeloMap(base_velo_front, odom_base);
	getBackVeloMap(base_velo_back, odom_base);

	// publish map
	pub_local_map_.publish(local_map_msg_);
	map_metadata_.publish(map_metadata_msg_);

	// reset map
	for (unsigned int jj = 0; jj < local_map_msg_.info.width * local_map_msg_.info.height; jj++)
	{
	    local_map_msg_.data[jj] = 0;
	}
    }
};


int main(int argc, char **argv)
{
    ros::init(argc, argv, "pc_to_local_map");
    ros::NodeHandle nh;
    tf::TransformListener listener;
    tf::StampedTransform transform_base_veloFront;
    tf::StampedTransform transform_base_veloBack;
    tf::StampedTransform transform_odom_base;

    std::clock_t start;
    double duration;

    // get first transform
    try
    {
	ros::Duration(1.0).sleep();
	listener.lookupTransform("/odom", "/base_link", ros::Time(0), transform_odom_base);
	listener.lookupTransform("/base_link", "/velodyne_front", ros::Time(0), transform_base_veloFront);
	listener.lookupTransform("/base_link", "/velodyne_back", ros::Time(0), transform_base_veloBack);
    }
    catch (tf::TransformException &ex)
    {
	ROS_ERROR("%s", ex.what());
	ros::Duration(1.0).sleep();
    }

    // set cell parameters
    float cell_size = 0.05f;
    // 20m * 20m
    unsigned int width = 400;
    unsigned height = 400;

    PointCloudToMap local_map(nh, cell_size, width, height);
    ros::Subscriber subFrontVelo = nh.subscribe("/velodyne_points_front", 10, &PointCloudToMap::getFrontVelodyneCallback, &local_map);
    ros::Subscriber subBackVelo = nh.subscribe("/velodyne_points_back", 10, &PointCloudToMap::getBackVelodyneCallback, &local_map);

    // velodyne sim has 10hz
    ros::Rate loop_rate(8);

    while (ros::ok())
    {
	ros::spinOnce();

	try
	{
	    listener.lookupTransform("/odom", "/base_link", ros::Time(0), transform_odom_base);
	    listener.lookupTransform("/base_link", "/velodyne_front", ros::Time(0), transform_base_veloFront);
	    listener.lookupTransform("/base_link", "/velodyne_back", ros::Time(0), transform_base_veloBack);
	}
	catch (tf::TransformException &ex)
	{
	    ROS_ERROR("%s", ex.what());
	    ros::Duration(1.0).sleep();
	}

	start = std::clock();
    local_map.publishLocalMap(transform_odom_base, transform_base_veloFront, transform_base_veloBack);
	duration = ( std::clock() - start ) / double(CLOCKS_PER_SEC);

	ROS_INFO("Running time: %f", duration);

	loop_rate.sleep();

	loop_rate.sleep();
    }
    // Exit
    return 0;
}
