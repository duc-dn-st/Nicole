/*
 * ===============================================================================
 * slope_bump_detection_node.h
 * Author: Schaefle Tobias
 * Date: 14.07.20
 * Email: tobias.schaefle@gmail.com
 * -------------------------------------------------------------------------------
 * Description:
 * This is the node for running the slope and bump detection algorithm.
 * ===============================================================================
 */

#include "../include/perception/slope_bump_detection.h"

#include <ros/ros.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "slope_bump_detection");
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");

    SlopeBumpDetection detection(nh, private_nh);

    ros::Subscriber sub_odom = nh.subscribe("/odometry/filtered", 1, &SlopeBumpDetection::odometryCallback, &detection);
    ros::Subscriber sub_velo = nh.subscribe("velodyne_points_front", 1, &SlopeBumpDetection::frontVelodynePointCloudCallback, &detection);
    std::cout << "Fix frame name for velodynes." << std::endl;
    // velodyne sim has 10hz
    ros::Rate loop_rate(5);

    while (ros::ok())
    {
        ros::spinOnce();

        detection.publishSlopeBumpMap();

        loop_rate.sleep();
    }

    return 0;
}
