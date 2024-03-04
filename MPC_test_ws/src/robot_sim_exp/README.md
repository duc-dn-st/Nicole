# Robot Main
This package is the task manager of the robot. It receives the trajectory and the information of the histogram map and the local map. Based on the current situation it will decide if the robot has to do an emergency stop or can continue moving along the trajectory or avoiding an obstacle. If emergency stop will take please a boolean will be set to true and the controller receives the information via a Service and stops the robot.  
Several launch files are included in this package to run the robot.

## How to run
The main call can be called with  
*rosrun robot_sim_exp sdv_main*
The launch files for running either experiment or simulation can be run with  
*roslaunch robot_sim_exp run_experiment.launch lidar:=true*  
*roslaunch robot_sim_exp run_sim.launch lidar:=true*  
At the moment *lidar* should be set to true.