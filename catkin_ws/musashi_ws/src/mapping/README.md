# Mapping
This package contains several nodes to generate a local map or inflate a static map. The inflation is done by dividing the robot into 4 equal sized squares and 1 recangle, the radius of the circle covering one square and the radius of the circle covering the rectangle is used to inflate the obstacles. The inflated map will have different costs for the radii. Collision is checked by checking if the center of the squares or the center of the rectangle overlaps with the respective inflated value of the obstacle.

## How to run
The inflation of the static obstacles can be called with  
*rosrun mapping inflate_static_map_node*  
The local map generation can be called with  
*rosrun mapping pc_to_local_mape_node*  
for the local map with inflation and without can be called with  
*rosrun mapping pc_to_local_map_no_inflation_node*