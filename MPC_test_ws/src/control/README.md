# ControllerNode

## Brief explanation
The main program of the controller module receives information from the task manager regarding new trajectories, collision avoidance flag or a emergency flag in a seperate thread. This thread also calls the collision avoidance algorithm if the flag is active. In that case the collision avoidance algorithm will send a new reference trajectory for avoiding the obstacle(s).
In a second thread is the call of the controller. The controller itself is in a seperate class and the main has an object of it. The controller calculates the new commands and sends them to the actuators.  
At the moment several controllers are implemented in this package. A model predictive controller (MPC) following the optimal trajectory, a vector field histogram (VFH) pure pursuit controller that follows the path of the generated trajectory and avoids simulatinously obstacles along the path, an open loop controller and a controller for pseudo holonomic mobile robots following a trajectory. At the moment each controller can be selected by changing code in the "controller_node" file.  
A controller for the steering and driving motors is provided that turns the wheels to the desired angle or desired driving velocities provided by the path or trajectory following controllers.
## How to run it
Enter in terminal or input it in a launch file:  
"rosrun control controller_node" to run the controller following the trajectory.  
The motor controllers can be called with "rosrun control low_level_controller".

## Task manager input (service)
Receives new information about the current state of the system (follow trajectory or new trajectory, collision avoidance active, emergency stop). The information is send via flags and a trajectory array. The controller main handles the cases via and sends the commands to the controller.

## Collision avoidance input (service or action server)
Calls the collision avoidance algorithm (currently VFH) and receives new reference points.

## Subscribing messages
In seperates threads the main receives messages from the steering motor encoders and the driving motor encoders. Furthermore, it subscribes to the localization topic. All messages will be fed to the function call of the controller. It also subscribes to the VFH path, current linear velocity and goal.

# Controller

## Brief
The call to the controller should have as input the new encoder values for the steering and driving motors, the current pose of the robot and the new reference point for the robot. A controller base class is provided which eases the handling with gazebo or the real system. Depending on the controller other inputs to the controller are also given.

## Structure of every controller
Every controller should be written as a class. The controller has to inherit the ControllerBase class! The base class takes as constructor inputs a NodeHandle nh, a private Nodehandle private_nh, transformations from the baselink to each of the wheels (trans_fl, trans_fr, trans_bl, trans_br). Therefore a controller constructor should look as follows:
- < ControllerName >(ros::NodeHandle nh, ros::NodeHandle private_nh, tf::StampedTransform trans_fl, tf::StampedTransform trans_fr,  tf::StampedTransform trans_bl, tf::StampedTransform trans_br, < controller input >)

The transfroms can be accessed as:
- trans_fl_
- trans_fr_
- trans_bl_
- trans_br_

The ControllerBase class provides functions to send commands to the motor controllers or simulation and a function to stop the motion of the robot. The stop motion function is public and the function for sending commands is protected.
The names and inputs of the functions are as follows:
- stopMotion()
- sendCommands(const double wheel_cmd_fl, const double wheel_cmd_fr, const double wheel_cmd_bl, const double wheel_cmd_br, const double steering_cmd_fl, const double steering_cmd_fr, const double steering_cmd_bl, const double steering_cmd_br)

The wheel commands are in [m/s] and the steering commands are in [rad].

## Controller parameter


## Select controller
In "utilities.h" several different settings (mainly for testing) can be selected to run a controller. In the namespace "Testing" the controllers (MPC, open loop and VFH/PP) can be selected. Furthermore, the boolean "TEST_AVOIDANCE" will ignore the emergency stop such that only avoidance will run. The boolean "TRACKING_ONLY" will not avoid obstacles but only focuses on tracking the path (only for VFH/PP).

# Motor Controller

## Brief
The motor controllers receive the desired linear velocity and steering angle from the controllers (subscribe to the respective self made message). A PI controller will try to reach the desired values with a much shorter sampling time than the tracking controllers.  
The gains of the motor controllers can be changed in "low_level_controller.h". In the future those should be changed via dynamic reconfigure.

## Testing
The motor controllers can be tested with rqt_gui in ROS. Publish desired steering angles or driving velocities to the sdv_msgs::ControlReference message and motors will move to the desired values.