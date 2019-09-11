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

#ifndef FRENET_PLANNER_H
#define FRENET_PLANNER_H


#include <geometry_msgs/TransformStamped.h>


//headers in Eigen
#include <Eigen/Core>


struct Point;


namespace autoware_msgs
{
  ROS_DECLARE_MESSAGE(Lane); 
  ROS_DECLARE_MESSAGE(DetectedObjectArray); 
}

struct FrenetPoint
{
  Eigen::Vector4d s_state;
  Eigen::Vector4d d_state;
};

enum class ReferenceType
{
  Waypoint,
  Obstacle,
  StopLine,
  AvoidingPoint,
  Unknown
};

struct ReferenceTypeInfo
{
  ReferenceType type;
  size_t referencing_object_id;
  size_t referencing_object_index;
};

struct ReferencePoint
{
  FrenetPoint frenet_point;
  double lateral_max_offset;
  double lateral_sampling_resolution;
  double longitudinal_max_offset;
  double longitudinal_sampling_resolution;
};

struct TrajecotoryPoint
{
  double x;
  double y;
  double yaw;
  double curvature;
  double velocity;
  double accerelation;
};

//TODO: change name to Trajectory
struct Trajectory
{
  //deprecated in v0.4
  autoware_msgs::Lane trajectory_points;
  std::vector<FrenetPoint> frenet_trajectory_points;
  std::vector<TrajecotoryPoint> calculated_trajectory_points;
  double required_time;
};




class FrenetPlanner
{
public:

  FrenetPlanner(
    double initial_velocity_ms,
    double velocity_before_obstalcle_ms,
    double distance_before_obstalcle,
    double obstacle_radius_from_center_point,
    double min_lateral_referencing_offset_for_avoidance,
    double max_lateral_referencing_offset_for_avoidance,
    double diff_waypoints_cost_coef,
    double diff_last_waypoint_cost_coef,
    double jerk_cost_coef,
    double required_time_cost_coef,
    double comfort_acceleration_cost_coef,
    double lookahead_distance_per_ms_for_reference_point,
    double converge_distance_per_ms_for_stopline,
    double linear_velocity);
  ~FrenetPlanner();
  
  
  void doPlan(const geometry_msgs::PoseStamped& in_current_pose,
              const geometry_msgs::TwistStamped& in_current_twist,
              const std::vector<Point>& in_nearest_lane_points,
              const std::vector<autoware_msgs::Waypoint>& in_reference_waypoints,
              const std::unique_ptr<autoware_msgs::DetectedObjectArray>& in_objects_ptr,
              autoware_msgs::Lane& out_trajectory,
              std::vector<autoware_msgs::Lane>& out_debug_trajectories,
              std::vector<geometry_msgs::Point>& out_reference_points);
  
  
private:
  
  //initialize with rosparam
  double initial_velocity_ms_;
  double velcity_ms_before_obstalcle_;
  double distance_before_obstalcle_;
  double obstacle_radius_from_center_point_;
  double min_lateral_referencing_offset_for_avoidance_;
  double max_lateral_referencing_offset_for_avoidance_;
  double diff_waypoints_cost_coef_;
  double diff_last_waypoint_cost_coef_;
  double jerk_cost_coef_;
  double required_time_cost_coef_;
  double comfort_acceleration_cost_coef_;
  
  double lookahead_distance_per_ms_for_reference_point_;
  double minimum_lookahead_distance_for_reference_point_;
  double lookahead_distance_for_reference_point_;
  
  double converge_distance_per_ms_for_stop_;
  double radius_from_reference_point_for_valid_trajectory_;
  double dt_for_sampling_points_;
  
  double linear_velocity_;
  // TODO: think better name previous_best_trajectoy?
  std::unique_ptr<Trajectory> kept_current_trajectory_;
  std::unique_ptr<Trajectory> kept_next_trajectory_;
  
  
  std::unique_ptr<ReferencePoint> kept_current_reference_point_;
  std::unique_ptr<ReferencePoint> kept_next_reference_point_;
  
  std::unique_ptr<std::vector<autoware_msgs::Waypoint>> previous_best_path_;
  
  bool generateEntirePath(
    const geometry_msgs::PoseStamped& current_pose,
    const std::vector<Point>& lane_points,
    const std::vector<autoware_msgs::Waypoint>& reference_waypoints,
    const std::unique_ptr<autoware_msgs::DetectedObjectArray>& in_objects_ptr,
    std::vector<autoware_msgs::Waypoint>& path_points,
    std::vector<autoware_msgs::Lane>& out_debug_trajectories,
    std::vector<geometry_msgs::Point>& out_reference_points
    );

  void  getNearestPoints(const geometry_msgs::Point& point,
                        const std::vector<Point>& nearest_lane_points,
                        Point& nearest_point,
                        Point& second_nearest_point);
                        
  void  getNearestPoint(const geometry_msgs::Point& point,
                        const std::vector<Point>& nearest_lane_points,
                        Point& nearest_point);
  
  void getNearestWaypoint(const geometry_msgs::Point& point,
                          const  std::vector<autoware_msgs::Waypoint>& waypoints,
                          autoware_msgs::Waypoint& nearest_waypoint);
                        
  void getNearestWaypoints(const geometry_msgs::Pose& point,
                          const autoware_msgs::Lane& waypoints,
                          autoware_msgs::Waypoint& nearest_waypoint,
                          autoware_msgs::Waypoint& second_nearest_waypoint);
               
  bool generateTrajectory(
    const geometry_msgs::Pose& ego_pose,
    const std::vector<Point>& lane_points,
    const std::vector<autoware_msgs::Waypoint>& reference_waypoints,  
    const FrenetPoint& origin_frenet_point,
    const FrenetPoint& reference_freent_point,
    const double time_horizon,
    const double dt_for_sampling_points, 
    Trajectory& trajectory);
    
    
  bool calculateWaypoint(const std::vector<Point>& lane_points, 
                        const FrenetPoint& frenet_point,
                        autoware_msgs::Waypoint& waypoint);
  
  bool calculateTrajectoryPoint(const std::vector<Point>& lane_points, 
                           const FrenetPoint& frenet_point,
                           TrajecotoryPoint& waypoint);
  
  //TODO: think better name for delta_s
  //TODO: improbe by changing delta_yaw(reference current or previous)
  bool convertFrenetPosition2CartesianPosition(
                           const double frenet_s,
                           const double frenet_d,
                           const geometry_msgs::Point& nearest_lane_cartesian_point,
                           const double delta_s,
                           const double delta_yaw,
                           geometry_msgs::Point& waypoint_position);
                           
  bool convertCartesianPosition2FrenetPosition(
        const geometry_msgs::Point& cartesian_point,
        const std::vector<Point> lane_points,        
        double& frenet_s_position,
        double& frenet_d_position);
        
  bool selectBestTrajectory(
    const std::vector<Trajectory>& trajectories,
    const std::unique_ptr<autoware_msgs::DetectedObjectArray>& objects_ptr,
    const std::vector<autoware_msgs::Waypoint>& cropped_reference_waypoints,
    std::unique_ptr<ReferencePoint>& kept_reference_point,    
    std::unique_ptr<Trajectory>& kept_best_trajectory);
  
  bool isCollision(const autoware_msgs::Waypoint& waypoint,
                   const autoware_msgs::DetectedObjectArray& objects);
                   
  bool isCollision(const autoware_msgs::Waypoint& waypoint,
                   const autoware_msgs::DetectedObjectArray& objects,
                   size_t& collision_object_id,
                   size_t& collision_object_index);
                      
  bool drawTrajectories(
              const geometry_msgs::Pose& ego_pose,
              const FrenetPoint& frenet_current_point,
              const ReferencePoint& reference_point,
              const std::vector<Point>& in_nearest_lane_points,
              const std::vector<autoware_msgs::Waypoint>& in_reference_waypoints,
              std::vector<Trajectory>& trajectories,
              std::vector<autoware_msgs::Lane>& out_debug_trajectories);
    
  bool isTrajectoryCollisionFree(
    const std::vector<autoware_msgs::Waypoint>& trajectory_points,
    const autoware_msgs::DetectedObjectArray& objects,
    size_t& collision_waypoint_index,
    size_t& collision_object_id,
    size_t& collision_object_index);
    
  bool isTrajectoryCollisionFree(
    const std::vector<autoware_msgs::Waypoint>& trajectory_points,
    const autoware_msgs::DetectedObjectArray& objects);
    
  void getNearestWaypointIndex(const geometry_msgs::Point& point,
                                    const std::vector<autoware_msgs::Waypoint>& waypoints,
                                    size_t& nearest_waypoint_index);
};

#endif
