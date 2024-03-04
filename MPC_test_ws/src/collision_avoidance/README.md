# Obstacle Avoidance

## Brief explanation
This package contains a node that implemented the VFH algorithm and a class that calculates the range to obstacles ("obstacle_range.h"). 

## How to run it
The VFH runs with the command:  
"rosrun collision_avoidance obstacle_avoidance"

## VFH subscribe
VFH subscribes to the current robot pose, the inflated local map, and the goal point along the generated optimal trajectory.  At the moment of writing the VFH does not include the slope map in the generation of a safe motion direction.

## VFH publish
The VFH algorithm publishes the polar histogram map, a VFH trajectory, a vector pointing at direction VFH wants to go and a path also pointing in the desired motion direction.

## VFH parameters
The parameters of VFH can be adjusted using a .yaml file such as "vfh_experiment_params.yaml".