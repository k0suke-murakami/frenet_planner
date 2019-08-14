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
  Eigen::Vector3d s_state;
  Eigen::Vector3d d_state;
};

enum class ReferenceType
{
  Waypoint,
  AvoidableStaticObstacle,
  NonAvoidableStaticObstacle,
  StopLine
};

struct ReferencePoint
{
  FrenetPoint frenet_point;
  geometry_msgs::Point cartesian_point;
  double lateral_offset;
  double lateral_sampling_resolution;
  double longutudinal_offset;
  double longutudinal_sampling_resolution;
  double time_horizon;
  double time_horizon_offset;
  double time_horizon_sampling_resolution;
  ReferenceType reference_type;
};

//TODO: change name to Trajectory
struct Trajectory
{
  autoware_msgs::Lane trajectory_points;
  std::vector<FrenetPoint> frenet_trajectory_points;
  // TODO: think better implementation
  Eigen::Vector3d target_d;
  // double cost;
};



class FrenetPlanner
{
public:

  FrenetPlanner();
  ~FrenetPlanner();
  
  
  void doPlan(const geometry_msgs::PoseStamped& in_current_pose,
              const geometry_msgs::TwistStamped& in_current_twist,
              const std::vector<Point>& in_nearest_lane_points,
              const std::vector<autoware_msgs::Waypoint>& in_reference_waypoints,
              const autoware_msgs::DetectedObjectArray& in_objects,
              const geometry_msgs::TransformStamped& in_wp2map_tf,
              autoware_msgs::Lane& out_trajectory,
              std::vector<autoware_msgs::Lane>& out_debug_trajectories,
              std::vector<geometry_msgs::Point>& out_target_points);
  
  
private:
  int debug_change_traj_count_;
  double dt_for_sampling_points_;
  double search_radius_for_target_point_;
  
  bool is_initial_goal_;
  
  
  // TODO: think better name previous_best_trajectoy?
  std::unique_ptr<Trajectory> kept_current_trajectory_;
  std::unique_ptr<FrenetPoint> frenet_target_trajectory_point_;
  
  std::unique_ptr<Trajectory> kept_next_trajectory_;
  
  
  std::unique_ptr<ReferencePoint> kept_current_reference_point_;
  std::unique_ptr<ReferencePoint> kept_next_reference_point_;
  

  void  getNearestPoints(const geometry_msgs::Point& point,
                        const std::vector<Point>& nearest_lane_points,
                        Point& nearest_point,
                        Point& second_nearest_point);
  
  void getNearestWaypoint(const geometry_msgs::Point& point,
                          const  std::vector<autoware_msgs::Waypoint>& waypoints,
                          autoware_msgs::Waypoint& nearest_waypoint);
                        
  void getNearestWaypoints(const geometry_msgs::Pose& point,
                          const autoware_msgs::Lane& waypoints,
                          autoware_msgs::Waypoint& nearest_waypoint,
                          autoware_msgs::Waypoint& second_nearest_waypoint);
               
  bool getTrajectory(
    const std::vector<Point>& lane_points,
    const std::vector<autoware_msgs::Waypoint>& reference_waypoints,  
    const FrenetPoint& origin_frenet_point,
    const FrenetPoint& target_freent_point,
    const double time_horizon,
    const double dt_for_sampling_points, 
    Trajectory& trajectory);
    
    
  bool calculateWaypoint(const std::vector<Point>& lane_points, 
                        const double s_position,
                        const double s_velocity,
                        const double s_acceleration,
                        const double d_position,
                        const double d_velocity,
                        const double d_acceleration,
                        autoware_msgs::Waypoint& waypoint);
  
  //TODO: this implementation might habe bug
  //TODO: think better name for delta_s
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
  
    
  bool getBestTrajectory(
    const std::vector<Trajectory>& trajectories,
    const std::vector<Point>& lane_points,
    const autoware_msgs::DetectedObjectArray& objects,
    const std::vector<autoware_msgs::Waypoint>& cropped_reference_waypoints,
    const FrenetPoint& frenet_reference_trajectory_point,
    std::unique_ptr<Trajectory>& kept_best_trajectory);
  
  //assume the interface with behaior planner in new planning architecture
  bool containFlaggedWaypoint(const autoware_msgs::Lane& reference_waypoints,
                              autoware_msgs::Waypoint& flagged_waypoint);
                              
  bool isFlaggedWaypointCloseWithTrajectory(
        const autoware_msgs::Waypoint& flagged_waypoint_ptr,
        const std::vector<autoware_msgs::Waypoint>& waypoints);
        
  //TODO: change to more general name
  //TODO: pass pose for sophisticated collisiong check by using orientation
  bool isFlaggedWaypointCloseWithPoint(
        const autoware_msgs::Waypoint& flagged_waypoint_ptr,
        const geometry_msgs::Point& compared_point);
        
        
  // pick up target point from reference waypoints
  bool getTargetPoint(
       const geometry_msgs::Point& origin_cartesian_point,
       const double origin_linear_velocity,
       const std::vector<autoware_msgs::Waypoint>& waypoints,
       const std::vector<Point>& lane_points,
       ReferencePoint& reference_point
  );
  
  
  // pick up new target point from kept_trajectory/lane points
  bool updateTargetPoint(
    const std::unique_ptr<Trajectory>& kept_trajectory,
    const std::vector<autoware_msgs::Waypoint>& waypoints,
    const std::vector<Point>& lane_points,
    const autoware_msgs::DetectedObjectArray& objects,
    const std::unique_ptr<ReferencePoint>& kept_reference_point,
    ReferencePoint& reference_point);
    
  // Flagged Waypoint is defined in new planning architecture
  //TODO: is there any way to make cleaner interface than this
  bool includeFlaggedWaypoint(
    const std::vector<autoware_msgs::Waypoint>& waypoints,
    autoware_msgs::Waypoint& flagged_waypoints
  ); 
  
  bool isCollision(const autoware_msgs::Waypoint& waypoint,
                   const autoware_msgs::DetectedObjectArray& objects);
  
  bool isReferencePointValid(const geometry_msgs::Pose& ego_pose,
                          const geometry_msgs::Point& cartesian_target_point,
                          const geometry_msgs::Point& last_reference_waypoint);
                          
  bool drawTrajectories(
              const FrenetPoint& frenet_current_point,
              const ReferencePoint& reference_point,
              const std::vector<Point>& in_nearest_lane_points,
              const std::vector<autoware_msgs::Waypoint>& in_reference_waypoints,
              std::vector<Trajectory>& trajectories,
              std::vector<autoware_msgs::Lane>& out_debug_trajectories);
  
  bool getOriginPointAndTargetPoint(
    const geometry_msgs::Pose& ego_pose,
    const double ego_linear_velocity,
    const std::vector<autoware_msgs::Waypoint>& reference_waypoints,
    const std::vector<Point>& lane_points,
    const autoware_msgs::DetectedObjectArray& objects,
    std::unique_ptr<Trajectory>& kept_current_trajectory,  
    FrenetPoint& origin_point,
    std::unique_ptr<ReferencePoint>& current_target_point);
    
  bool getNextTargetPoint(
    const geometry_msgs::Pose& ego_pose,
    const double origin_linear_velocity,    
    const std::vector<autoware_msgs::Waypoint>& reference_waypoints,
    const std::vector<Point>& lane_points,
    const autoware_msgs::DetectedObjectArray& objects,
    std::unique_ptr<Trajectory>& kept_current_trajectory,
    std::unique_ptr<Trajectory>& kept_next_trajectory,
    std::unique_ptr<ReferencePoint>& current_target_point,
    std::unique_ptr<ReferencePoint>& next_target_point);
  
};

#endif
