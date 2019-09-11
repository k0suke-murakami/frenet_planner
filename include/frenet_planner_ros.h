/*
 * Copyright 2015-2019 Autoware Foundation. All rights reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef FRENET_PLANNER_ROS_H
#define FRENET_PLANNER_ROS_H

struct Point;

namespace tf2_ros
{
  class Buffer;
  class TransformListener;
}

class FrenetPlanner;
class VectorMap;
class CalculateCenterLine;
class ModifiedReferencePathGenerator;

namespace autoware_msgs
{
  ROS_DECLARE_MESSAGE(Lane); 
  ROS_DECLARE_MESSAGE(DetectedObjectArray); 
  ROS_DECLARE_MESSAGE(Waypoint); 
}

namespace grid_map_msgs
{
  ROS_DECLARE_MESSAGE(GridMap); 
}

namespace grid_map
{
  class GridMap;
}

namespace geometry_msgs
{ 
  ROS_DECLARE_MESSAGE(PoseStamped);
  ROS_DECLARE_MESSAGE(TwistStamped);
  ROS_DECLARE_MESSAGE(TransformStamped);
}

class FrenetPlannerROS
{
public:

  FrenetPlannerROS();
  ~FrenetPlannerROS();
  void run();

private:
  // ros
  ros::NodeHandle nh_, private_nh_;
  ros::Publisher optimized_waypoints_pub_;
  ros::Publisher markers_pub_;
  ros::Publisher gridmap_pointcloud_pub_;
  ros::Subscriber current_pose_sub_;
  ros::Subscriber current_velocity_sub_;
  ros::Subscriber final_waypoints_sub_;
  ros::Subscriber objects_sub_;
  ros::Subscriber grid_map_sub_;
  
  bool use_global_waypoints_as_center_line_;
  bool has_calculated_center_line_from_global_waypoints_;
  
  //TODO: not 
  bool got_modified_reference_path_;
  bool only_testing_modified_global_path_;
  
  //TODO: not good code
  std::vector<autoware_msgs::Waypoint> modified_reference_path_;
  std::vector<Point> center_line_points_;
  // std::vector<autoware_msgs::Waypoint> debug_bspline_path_;
  
  ros::Timer timer_;
  
  
  std::unique_ptr<tf2_ros::Buffer> tf2_buffer_ptr_;
  std::unique_ptr<tf2_ros::TransformListener> tf2_listner_ptr_;
  std::unique_ptr<geometry_msgs::TransformStamped> lidar2map_tf_;
  std::unique_ptr<geometry_msgs::TransformStamped> map2lidar_tf_;
  
  
  
  std::unique_ptr<autoware_msgs::Lane> in_waypoints_ptr_;
  std::unique_ptr<geometry_msgs::PoseStamped> in_pose_ptr_;
  std::unique_ptr<geometry_msgs::TwistStamped> in_twist_ptr_;
  std::unique_ptr<autoware_msgs::DetectedObjectArray> in_objects_ptr_;
  std::unique_ptr<grid_map_msgs::GridMap> in_gridmap_ptr_;
  
  std::unique_ptr<FrenetPlanner> frenet_planner_ptr_;
  std::unique_ptr<VectorMap> vectormap_load_ptr_;
  std::unique_ptr<CalculateCenterLine> calculate_center_line_ptr_;
  std::unique_ptr<ModifiedReferencePathGenerator> modified_reference_path_generator_ptr_;
  std::unique_ptr<std::vector<Point>> global_center_points_ptr_;
  
  void waypointsCallback(const autoware_msgs::Lane& msg);
  void currentPoseCallback(const geometry_msgs::PoseStamped& msg);
  void currentVelocityCallback(const geometry_msgs::TwistStamped& msg);
  void objectsCallback(const autoware_msgs::DetectedObjectArray& msg);
  void gridmapCallback(const grid_map_msgs::GridMap& msg);
  void timerCallback(const ros::TimerEvent &e);
  void loadVectormap();
  Point getNearestPoint(const geometry_msgs::PoseStamped& ego_pose);

};

#endif
