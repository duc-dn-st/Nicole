/*
 * ===============================================================================
 * robot_communication.cpp
 * Author: Schaefle Tobias
 * -------------------------------------------------------------------------------
 * Description:
 * this node starts a commuication with a serial interface with the robot
 * the robot sends informations to ROS (battery, encoder, etc.), this program allows
 * the user to send velocities to the left and right wheel of the robot
 *
 * Subscribes to:
 * - /industrial_robot/wheel_velocities
 * - or to /cmd_vel
 *
 * Publishes:
 * - Data type: sensor_msgs/BatteryState
 *		industrial_robot_qt/EncoderWheelVel
 * - Address: encoder
 *	      battery
 * ===============================================================================
 */

#ifndef TASP_H
#define TASP_H

class Tasp
{
private:
    double bt_points_;		// should be a list or an array
    int &grid_map_ptr_;

    // deletes the new pos from the list and adds the neighbours if needed
    void updateBTPoint(double new_pos[2], double grid_neighbours[3]);

    // this function returns the distance whihc can be travelled without hitting an obstacle
    double getTravelDistance(double current_pos[2], unsigned int direction);


public:
    Tasp(int &grid_map_ptr);
};

#endif // TASP_H
