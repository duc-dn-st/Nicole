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
