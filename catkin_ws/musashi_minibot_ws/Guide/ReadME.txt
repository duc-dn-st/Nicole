## Trajectory Generation
rosrun trajectory_generation generate_trajectory

-> Change waypoints in global_trajectory.cpp line 165~
-> Change path velocity in Utilities.h

## PurePursuit
-> Change final threshold in pure_puresuit.h (POS_THRESHOLD)
-> Change lookahead distance & velocity in utilities.h
-> Robot measurement(e.g wheel radius for rpm calculations) also changed in utilities.h