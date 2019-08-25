# Frenet Planner

### How to build
* colcon
  - `colcon build --packages-up-to frenet_planner`
  - `colcon build --packages-select frenet_planner`
* catkin_make
  - `catkin_make --pkg frenet_planner`


### How to launch

* From a sourced terminal:

    - `roslaunch frenet_planner frenet_planner.launch`
    - `roslaunch frenet_planner frenet_planner.launch initial_velocity_kmh:=2.1 velocity_kmh_before_obstalce:=1.0 distance_before_obstalce:=7.0 obstacle_radius_from_center_point:=3.0 min_lateral_referencing_offset_for_avoidance:=5.0 max_lateral_referencing_offset_for_avoidance:=8.0 cost_diff_waypoints_coef:=0.0 cost_diff_last_waypoint_coef:=1.0`


### Parameters

Launch file available parameters:

|Parameter| Type| Description|
----------|-----|--------
|`initial_velocity_kmh`|*Double* |Generate a trajectory which have this linear veocity at the first waypoint. Unit is `kmh`. Default `2.1`.|
|`velocity_kmh_before_obstalce`|*Double*|Target decelerated velocity when confronting a obstale. Unit is `kmh`. Default `1.0`.|
|`distance_before_obstacle`|*Double*|Target keep-away distance when facing a obstalce. Unit is `m`. Default `7.0`.|
|`obstalce_radius_from_center_point`|*Double*|Radius using for collision check. Unit is `m`. Default `3.0`.|
|`min_latetal_referencing_offset_for_avoidance`|*Double*|Minimum lateral offset for a reference point when performing an avoiding manuever. Unit is `m`. Default `5.0`.|
|`max_latetal_referencing_offset_for_avoidance`|*Double*|Maximum lateral offset for a reference point when performing an avoiding manuever. Unit is `m`. Default `8.0`.|
|`cost_diff_waypoints_coef`|*Double*|Coefficient for cost(running cost) calculation with reference waypoints. Scale from 0.0 to 1.0. Default `0.0`.|
|`cost_diff_last_waypoint_coef`|*Double*|Coefficient for cost(terminal cost) calculation with last reference waypoint. Scale from 0.0 to 1.0. Default `1.0`.|
|`lookahead_distance_per_kmh_for_reference_point`|*Double*|Ratio for deciding referece point based on current_velocity. Unit is `m/kmh`. Default `2.0`.|
|`converge_distance_per_kmh_for_stop`|*Double*|Ratio for deciding a distance at which starting the deceleration for stop. Unit is `m/kmh`. Default `2.36`.|


### Subscribed topics

|Topic|Type|Objective|
------|----|---------
|`/current_pose`|`geometry_msgs::PoseStamped`|WIP|
|`/current_velocity`|`geometry_msgs::TwistStamped`|WIP|
|`/base_waypoints`|`autoware_msgs::Lane`|WIP|
|`/detection/lidar_detector/objects`|`autoware_msgs::DetectedObjectArray`|WIP|

### Published topics

|Topic|Type|Objective|
------|----|---------
|`/final_waypoints`|`autoware_msgs::Lane`|WIP|
