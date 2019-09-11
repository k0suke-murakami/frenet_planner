/*
 * Copyright 2015-2019 Autoware Foundation. All rights reserved.
 *
 * Licensed under the Apacpoints_marker_arrayion 2.0 (the "License");
 * you may not use this file except in compliance with thxpxoe License.
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
#include <iostream>
#include <vector>
#include <chrono>


#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>
#include <tf2/utils.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>


//TODO: in order to visualize text velocity
#include <tf/transform_datatypes.h>
#include <geometry_msgs/TransformStamped.h>


#include <autoware_msgs/Lane.h>
#include <autoware_msgs/DetectedObjectArray.h>

#include <grid_map_ros/GridMapRosConverter.hpp>
#include <grid_map_ros/grid_map_ros.hpp>
#include <grid_map_msgs/GridMap.h>

#include "frenet_planner.h"
#include "vectormap_ros.h"
#include "vectormap_struct.h"
#include "calculate_center_line.h"
#include "modified_reference_path_generator.h"

#include "frenet_planner_ros.h"

FrenetPlannerROS::FrenetPlannerROS()
  : nh_(), 
  private_nh_("~"),
  use_global_waypoints_as_center_line_(true),
  has_calculated_center_line_from_global_waypoints_(false),
  got_modified_reference_path_(false)
{
  double timer_callback_delta_second;
  private_nh_.param<double>("timer_callback_delta_second", timer_callback_delta_second, 0.1);
  
  double initial_velocity_kmh;
  double velcity_kmh_before_obstalcle;
  double distance_before_obstalcle;
  double obstacle_radius_from_center_point;
  double min_lateral_referencing_offset_for_avoidance;
  double max_lateral_referencing_offset_for_avoidance;
  double diff_waypoints_cost_coef;
  double diff_last_waypoint_cost_coef;
  double jerk_cost_coef;
  double required_time_cost_coef;
  double comfort_acceleration_cost_coef;
  double lookahead_distance_per_kmh_for_reference_point;
  double converge_distance_per_kmh_for_stop;
  double linear_velocity_kmh;
  
  double min_radius;
  
  private_nh_.param<double>("initial_velocity_kmh", initial_velocity_kmh, 2.1);
  private_nh_.param<double>("velcity_kmh_before_obstalcle", velcity_kmh_before_obstalcle, 1.0);
  private_nh_.param<double>("distance_before_obstacle", distance_before_obstalcle, 7.0);
  private_nh_.param<double>("obstalce_radius_from_center_point", obstacle_radius_from_center_point, 4.0);
  private_nh_.param<double>("min_lateral_referencing_offset_for_avoidance", min_lateral_referencing_offset_for_avoidance, 5.0);
  private_nh_.param<double>("max_lateral_referencing_offset_for_avoidance", max_lateral_referencing_offset_for_avoidance, 8.0);
  private_nh_.param<double>("diff_waypoints_cost_coef", diff_waypoints_cost_coef, 0.0);
  private_nh_.param<double>("diff_last_waypoint_cost_coef", diff_last_waypoint_cost_coef, 1.0);
  private_nh_.param<double>("jerk_cost_coef", jerk_cost_coef, 0.25);
  private_nh_.param<double>("required_time_cost_coef", required_time_cost_coef, 1.0);
  private_nh_.param<double>("comfort_acceleration_cost_coef", comfort_acceleration_cost_coef, 0.0);
  private_nh_.param<double>("lookahead_distance_per_kmh_for_reference_point", lookahead_distance_per_kmh_for_reference_point, 2.0);
  private_nh_.param<double>("converge_distance_per_kmh_for_stop", converge_distance_per_kmh_for_stop, 2.36);
  private_nh_.param<double>("linear_velocity_kmh", linear_velocity_kmh, 5.0);
  private_nh_.param<bool>("only_testing_modified_global_path", only_testing_modified_global_path_, false);
  private_nh_.param<double>("min_radius", min_radius, 1.6);
  const double kmh2ms = 0.2778;
  const double initial_velocity_ms = initial_velocity_kmh * kmh2ms;
  const double velocity_ms_before_obstacle = velcity_kmh_before_obstalcle * kmh2ms;
  double lookahead_distance_per_ms_for_reference_point = lookahead_distance_per_kmh_for_reference_point/kmh2ms;
  double converge_distance_per_ms_for_stop = converge_distance_per_kmh_for_stop/kmh2ms;
  double linear_velocity_ms = linear_velocity_kmh*kmh2ms;
  frenet_planner_ptr_.reset(
    new FrenetPlanner(
        initial_velocity_ms,
        velocity_ms_before_obstacle,
        distance_before_obstalcle,
        obstacle_radius_from_center_point,
        min_lateral_referencing_offset_for_avoidance,
        max_lateral_referencing_offset_for_avoidance,
        diff_waypoints_cost_coef,
        diff_last_waypoint_cost_coef,
        jerk_cost_coef,
        required_time_cost_coef,
        comfort_acceleration_cost_coef,
        lookahead_distance_per_ms_for_reference_point,
        converge_distance_per_ms_for_stop,
        linear_velocity_ms));
  // TODO: assume that vectormap is already published when constructing FrenetPlannerROS
  if(!use_global_waypoints_as_center_line_)
  {
    vectormap_load_ptr_.reset(new VectorMap());
    loadVectormap();
  }
  else
  {
    calculate_center_line_ptr_.reset(new CalculateCenterLine());
  }
  modified_reference_path_generator_ptr_.reset(
    new ModifiedReferencePathGenerator(
      min_radius));
  
  tf2_buffer_ptr_.reset(new tf2_ros::Buffer());
  tf2_listner_ptr_.reset(new tf2_ros::TransformListener(*tf2_buffer_ptr_));
  
  
  optimized_waypoints_pub_ = nh_.advertise<autoware_msgs::Lane>("safety_waypoints", 1, true);
  markers_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("frenet_planner_debug_markes", 1, true);
  gridmap_pointcloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("gridmap_pointcloud", 1, true);
  final_waypoints_sub_ = nh_.subscribe("base_waypoints", 1, &FrenetPlannerROS::waypointsCallback, this);
  current_pose_sub_ = nh_.subscribe("/current_pose", 1, &FrenetPlannerROS::currentPoseCallback, this);
  current_velocity_sub_ = nh_.subscribe("/current_velocity", 1, &FrenetPlannerROS::currentVelocityCallback, this);
  objects_sub_ = nh_.subscribe("/detection/lidar_detector/objects", 1, &FrenetPlannerROS::objectsCallback, this);
  grid_map_sub_ = nh_.subscribe("/semantics/costmap", 1, &FrenetPlannerROS::gridmapCallback, this);
  // double timer_callback_dt = 0.05;
  // double timer_callback_dt = 0.1;
  // double timer_callback_dt = 1.0;
  // double timer_callback_dt = 0.5;
  timer_ = nh_.createTimer(ros::Duration(timer_callback_delta_second), &FrenetPlannerROS::timerCallback, this);
}

FrenetPlannerROS::~FrenetPlannerROS()
{
}


void FrenetPlannerROS::waypointsCallback(const autoware_msgs::Lane& msg)
{
  in_waypoints_ptr_.reset(new autoware_msgs::Lane(msg));
}

void FrenetPlannerROS::currentPoseCallback(const geometry_msgs::PoseStamped & msg)
{
  in_pose_ptr_.reset(new geometry_msgs::PoseStamped(msg));
}

void FrenetPlannerROS::currentVelocityCallback(const geometry_msgs::TwistStamped& msg)
{
  in_twist_ptr_.reset(new geometry_msgs::TwistStamped(msg));
}

void FrenetPlannerROS::gridmapCallback(const grid_map_msgs::GridMap& msg)
{ 
  if(in_waypoints_ptr_)
  {
    geometry_msgs::TransformStamped lidar2map_tf;
    geometry_msgs::TransformStamped map2lidar_tf;
    try
    {
        lidar2map_tf = tf2_buffer_ptr_->lookupTransform(
          /*target*/  in_waypoints_ptr_->header.frame_id, 
          /*src*/ msg.info.header.frame_id,
          ros::Time(0));
        lidar2map_tf_.reset(new geometry_msgs::TransformStamped(lidar2map_tf));
        map2lidar_tf = tf2_buffer_ptr_->lookupTransform(
          /*target*/  msg.info.header.frame_id, 
          /*src*/ in_waypoints_ptr_->header.frame_id,
          ros::Time(0));
        map2lidar_tf_.reset(new geometry_msgs::TransformStamped(map2lidar_tf));
    }
    catch (tf2::TransformException &ex)
    {
        ROS_WARN("%s", ex.what());
        return;
    }
    in_gridmap_ptr_.reset(new grid_map_msgs::GridMap(msg));
  }
}

void FrenetPlannerROS::objectsCallback(const autoware_msgs::DetectedObjectArray& msg)
{
  if(in_waypoints_ptr_)
  {
    if(msg.objects.size() == 0)
    {
      std::cerr << "ssize of objects is 0" << std::endl;
      return;
    }
    geometry_msgs::TransformStamped lidar2map_tf;
    try
    {
        lidar2map_tf = tf2_buffer_ptr_->lookupTransform(
          /*target*/  in_waypoints_ptr_->header.frame_id, 
          /*src*/ msg.header.frame_id,
          ros::Time(0));
        lidar2map_tf_.reset(new geometry_msgs::TransformStamped(lidar2map_tf));
    }
    catch (tf2::TransformException &ex)
    {
        ROS_WARN("%s", ex.what());
        return;
    }
    in_objects_ptr_.reset(new autoware_msgs::DetectedObjectArray(msg));
    in_objects_ptr_->header.frame_id = in_waypoints_ptr_->header.frame_id;
    for(auto& object: in_objects_ptr_->objects)
    {
      object.header.frame_id = in_waypoints_ptr_->header.frame_id;
      geometry_msgs::PoseStamped current_object_pose;
      current_object_pose.header = object.header;
      current_object_pose.pose = object.pose;
      geometry_msgs::PoseStamped transformed_pose;
      tf2::doTransform(current_object_pose, transformed_pose, *lidar2map_tf_);
      object.pose = transformed_pose.pose;
    }
  }
}

void FrenetPlannerROS::timerCallback(const ros::TimerEvent &e)
{
  if(!in_pose_ptr_)
  {
    std::cerr << "pose not arrive" << std::endl;
  }
  if(!in_twist_ptr_)
  {
    std::cerr << "twist not arrive" << std::endl;
  }
  if(!in_waypoints_ptr_)
  {
    std::cerr << "waypoints not arrive" << std::endl;
  }
  if(!in_gridmap_ptr_)
  {
    std::cerr << "costmap not arrive" << std::endl;
  }
  
  if(in_pose_ptr_ && 
     in_twist_ptr_ && 
     in_waypoints_ptr_ && 
     in_gridmap_ptr_) 
  { 
    // 1. 現在日時を取得
    std::chrono::high_resolution_clock::time_point begin = std::chrono::high_resolution_clock::now();
    
    grid_map::GridMap grid_map;
    grid_map::GridMapRosConverter::fromMessage(*in_gridmap_ptr_, grid_map);
    // std::vector<autoware_msgs::Waypoint> modified_reference_path;
    std::vector<autoware_msgs::Waypoint> debug_modified_smoothed_reference_path;
    std::vector<autoware_msgs::Waypoint> debug_bspline_path;
    sensor_msgs::PointCloud2 debug_clearance_map_pointcloud;
    // got_modified_reference_path_ = false;
    if(!got_modified_reference_path_)
    {
      //TODO: make it better by using time-series data
      //make local waypoints based on current_pose
      std::vector<autoware_msgs::Waypoint> local_reference_waypoints;
      double min_dist = 99999;
      size_t closest_wp_index = 0;
      for (size_t i = 0; i < in_waypoints_ptr_->waypoints.size(); i++)
      {
        double dx = in_waypoints_ptr_->waypoints[i].pose.pose.position.x - in_pose_ptr_->pose.position.x;
        double dy = in_waypoints_ptr_->waypoints[i].pose.pose.position.y - in_pose_ptr_->pose.position.y;
        double distance = std::sqrt(std::pow(dx, 2)+std::pow(dy,2));
        if(distance < min_dist)
        {
          min_dist = distance;
          closest_wp_index = i;
        }
      }
      //TODO: think better way
      for(size_t i = closest_wp_index; i< in_waypoints_ptr_->waypoints.size(); i++)
      {
        local_reference_waypoints.push_back(in_waypoints_ptr_->waypoints[i]);
      }
      
      
      double min_dist_from_goal = 99999;
      const double search_distance = 45;
      size_t closest_goal_wp_index = 0;
      for (size_t i = 0; i < local_reference_waypoints.size(); i++)
      {
        double dx = local_reference_waypoints[i].pose.pose.position.x - in_pose_ptr_->pose.position.x;
        double dy = local_reference_waypoints[i].pose.pose.position.y - in_pose_ptr_->pose.position.y;
        double distance = std::sqrt(std::pow(dx, 2)+std::pow(dy,2));
        if(distance < min_dist_from_goal && distance > search_distance)
        {
          min_dist_from_goal = distance;
          closest_goal_wp_index = i;
        }
      }
      geometry_msgs::Point goal_point = local_reference_waypoints[closest_goal_wp_index].pose.pose.position;
      geometry_msgs::Point start_point = in_pose_ptr_->pose.position;
      
      
      if(only_testing_modified_global_path_)
      {
        modified_reference_path_.clear();
      }
      got_modified_reference_path_ =  
        modified_reference_path_generator_ptr_->generateModifiedReferencePath(
            grid_map,
            start_point,
            goal_point,
            *lidar2map_tf_,
            *map2lidar_tf_,
            modified_reference_path_,
            debug_modified_smoothed_reference_path,
            debug_bspline_path,
            debug_clearance_map_pointcloud);
      if(!got_modified_reference_path_)
      { 
        std::vector<autoware_msgs::Waypoint> aaa;
        modified_reference_path_ = aaa;
        return;
      }
      
      if(only_testing_modified_global_path_)
      {
        got_modified_reference_path_ = false;
        std::cerr << "modified size " << modified_reference_path_.size() << std::endl;
        std::cerr << "bspline size " << debug_bspline_path.size() << std::endl;
      }
      
      // std::vector<Point> center_line_points;
      center_line_points_
      = calculate_center_line_ptr_->calculateCenterLineFromGlobalWaypoints(
        modified_reference_path_);
    }
    debug_clearance_map_pointcloud.header = in_gridmap_ptr_->info.header;
    gridmap_pointcloud_pub_.publish(debug_clearance_map_pointcloud);
     
    // 3. 現在日時を再度取得
    std::chrono::high_resolution_clock::time_point distance_end = std::chrono::high_resolution_clock::now();
    // 経過時間を取得
    std::chrono::nanoseconds elapsed_time = std::chrono::duration_cast<std::chrono::nanoseconds>(distance_end - begin);
    std::cout <<"distance transform " <<elapsed_time.count()/(1000.0*1000.0)<< " milli sec" << std::endl;
    
    
    autoware_msgs::Lane out_trajectory;
    std::vector<autoware_msgs::Lane> out_debug_trajectories;
    std::vector<geometry_msgs::Point> out_target_points;
    if(!only_testing_modified_global_path_)
    {
      std::vector<autoware_msgs::Waypoint> local_reference_waypoints;
      double min_dist = 99999;
      size_t closest_wp_index = 0;
      for (size_t i = 0; i < modified_reference_path_.size(); i++)
      {
        double dx = modified_reference_path_[i].pose.pose.position.x - in_pose_ptr_->pose.position.x;
        double dy = modified_reference_path_[i].pose.pose.position.y - in_pose_ptr_->pose.position.y;
        double distance = std::sqrt(std::pow(dx, 2)+std::pow(dy,2));
        if(distance < min_dist)
        {
          min_dist = distance;
          closest_wp_index = i;
        }
      }
      //TODO: think better way
      for(size_t i = closest_wp_index; i< modified_reference_path_.size(); i++)
      {
        local_reference_waypoints.push_back(modified_reference_path_[i]);
      }
      
      std::vector<Point> local_center_points;
      double min_dist2 = 99999;
      size_t closest_point_index = 0;
      for (size_t i = 0; i < center_line_points_.size(); i++)
      {
        double dx = center_line_points_[i].tx - in_pose_ptr_->pose.position.x;
        double dy = center_line_points_[i].ty - in_pose_ptr_->pose.position.y;
        double distance = std::sqrt(std::pow(dx, 2)+std::pow(dy,2));
        if(distance < min_dist)
        {
          min_dist2 = distance;
          closest_point_index = i;
        }
      }
      //TODO: think better way
      for(size_t i = closest_point_index; i< center_line_points_.size(); i++)
      {
        local_center_points.push_back(center_line_points_[i]);
      }
      
      
      // // TODO: somehow improve interface
      frenet_planner_ptr_->doPlan(*in_pose_ptr_, 
                                  *in_twist_ptr_, 
                                  local_center_points, 
                                  local_reference_waypoints,
                                  in_objects_ptr_,
                                  out_trajectory,
                                  out_debug_trajectories,
                                  out_target_points);
      std::cerr << "------"  << std::endl;
      // for(auto& trajectory: out_trajectory.waypoints)
      // {
      //   std::cerr << "trajectory " << trajectory.pose.pose.position.x << std::endl;
      //   std::cerr << "trajectory " << trajectory.pose.pose.position.y << std::endl;
      //   // std::cerr << "trajector twist " << trajectory.twist.twist.linear.x << std::endl;
      //   // trajectory.twist.twist.linear.x = 1.38;
      // }
      //3. 現在日時を再度取得
      std::chrono::high_resolution_clock::time_point path_end = std::chrono::high_resolution_clock::now();

      // 経過時間を取得
      std::chrono::nanoseconds elapsed_time2 = std::chrono::duration_cast<std::chrono::nanoseconds>(path_end - distance_end);
      std::cout << "path generation "<<elapsed_time2.count()/(1000.0*1000.0)<< " milli sec" << std::endl;
      // std::cerr << "output num wps" << out_trajectory.waypoints.size() << std::endl;
      std::cerr << "------------"  << std::endl;
      
      autoware_msgs::Lane dummy_lane = *in_waypoints_ptr_;
      dummy_lane.waypoints = local_reference_waypoints;
      dummy_lane.waypoints = out_trajectory.waypoints;
      optimized_waypoints_pub_.publish(dummy_lane);
    }
    // optimized_waypoints_pub_.publish(out_trajectory);
    
    
    
    //debug; marker array
    visualization_msgs::MarkerArray points_marker_array;
    int unique_id = 0;
    
    // // visualize debug target point
    // visualization_msgs::Marker debug_target_point;
    // debug_target_point.lifetime = ros::Duration(0.2);
    // debug_target_point.header = in_pose_ptr_->header;
    // debug_target_point.ns = std::string("debug_target_point_marker");
    // debug_target_point.action = visualization_msgs::Marker::MODIFY;
    // debug_target_point.pose.orientation.w = 1.0;
    // debug_target_point.id = unique_id;
    // debug_target_point.type = visualization_msgs::Marker::SPHERE_LIST;
    // debug_target_point.scale.x = 0.9;
    // debug_target_point.color.r = 1.0f;
    // debug_target_point.color.g = 1.0f;
    // debug_target_point.color.a = 1;
    // for(const auto& point: out_target_points)
    // {
    //   debug_target_point.points.push_back(point);
    // }
    // points_marker_array.markers.push_back(debug_target_point);
    // unique_id++;
    
    // visualize debug modified reference point
    visualization_msgs::Marker debug_modified_reference_points;
    debug_modified_reference_points.lifetime = ros::Duration(0.2);
    debug_modified_reference_points.header = in_pose_ptr_->header;
    debug_modified_reference_points.ns = std::string("debug_modified_reference_points");
    debug_modified_reference_points.action = visualization_msgs::Marker::MODIFY;
    debug_modified_reference_points.pose.orientation.w = 1.0;
    debug_modified_reference_points.id = unique_id;
    debug_modified_reference_points.type = visualization_msgs::Marker::SPHERE_LIST;
    debug_modified_reference_points.scale.x = 0.9;
    debug_modified_reference_points.color.r = 1.0f;
    debug_modified_reference_points.color.g = 1.0f;
    debug_modified_reference_points.color.a = 1;
    for(const auto& waypoint: modified_reference_path_)
    {
      debug_modified_reference_points.points.push_back(waypoint.pose.pose.position);
    }
    points_marker_array.markers.push_back(debug_modified_reference_points);
    unique_id++;
    
     // visualize debug modified smoothed reference point
    visualization_msgs::Marker debug_modified_smoothed_reference_points;
    debug_modified_smoothed_reference_points.lifetime = ros::Duration(0.2);
    debug_modified_smoothed_reference_points.header = in_pose_ptr_->header;
    debug_modified_smoothed_reference_points.ns = std::string("debug_modified_smoothed_reference_points");
    debug_modified_smoothed_reference_points.action = visualization_msgs::Marker::MODIFY;
    debug_modified_smoothed_reference_points.pose.orientation.w = 1.0;
    debug_modified_smoothed_reference_points.id = unique_id;
    debug_modified_smoothed_reference_points.type = visualization_msgs::Marker::SPHERE_LIST;
    debug_modified_smoothed_reference_points.scale.x = 0.9;
    debug_modified_smoothed_reference_points.color.r = 1.0f;
    debug_modified_smoothed_reference_points.color.a = 0.6;
    for(const auto& waypoint: debug_modified_smoothed_reference_path)
    {
      debug_modified_smoothed_reference_points.points.push_back(waypoint.pose.pose.position);
    }
    points_marker_array.markers.push_back(debug_modified_smoothed_reference_points);
    unique_id++;
    
     // visualize debug bspline
    visualization_msgs::Marker debug_bspline_path_marker;
    debug_bspline_path_marker.lifetime = ros::Duration(0.2);
    debug_bspline_path_marker.header = in_pose_ptr_->header;
    debug_bspline_path_marker.ns = std::string("debug_bspline_path_marker");
    debug_bspline_path_marker.action = visualization_msgs::Marker::MODIFY;
    debug_bspline_path_marker.pose.orientation.w = 1.0;
    debug_bspline_path_marker.id = unique_id;
    debug_bspline_path_marker.type = visualization_msgs::Marker::SPHERE_LIST;
    debug_bspline_path_marker.scale.x = 0.9;
    debug_bspline_path_marker.color.r = 1.0f;
    debug_bspline_path_marker.color.a = 1;
    for(const auto& waypoint: debug_bspline_path)
    {
      debug_bspline_path_marker.points.push_back(waypoint.pose.pose.position);
    }
    points_marker_array.markers.push_back(debug_bspline_path_marker);
    unique_id++;
    
    //text modified reference path curvature
    for (const auto& point: debug_modified_smoothed_reference_path)
    {
      visualization_msgs::Marker debuf_modified_curvature_text;
      debuf_modified_curvature_text.lifetime = ros::Duration(0.2);
      debuf_modified_curvature_text.header = in_pose_ptr_->header;
      debuf_modified_curvature_text.ns = std::string("debuf_modified_curvature_text");
      debuf_modified_curvature_text.action = visualization_msgs::Marker::ADD;
      debuf_modified_curvature_text.id = unique_id;
      debuf_modified_curvature_text.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
      debuf_modified_curvature_text.scale.x = 1;
      debuf_modified_curvature_text.scale.y = 0.1;
      debuf_modified_curvature_text.scale.z = 0.4;

      // texts are green
      debuf_modified_curvature_text.color.g = 1.0f;
      debuf_modified_curvature_text.color.a = 1.0;
      
      
      geometry_msgs::Point relative_p;
      relative_p.y = 0.8;
      geometry_msgs::Pose pose;
      pose.position = point.pose.pose.position;
      pose.orientation.x = 0;
      pose.orientation.y = 0;
      pose.orientation.z = 0;
      pose.orientation.w = 1.0;
      tf::Transform inverse;
      tf::poseMsgToTF(pose, inverse);

      tf::Point p;
      pointMsgToTF(relative_p, p);
      tf::Point tf_p = inverse * p;
      geometry_msgs::Point tf_point_msg;
      pointTFToMsg(tf_p, tf_point_msg);
      debuf_modified_curvature_text.pose.position = tf_point_msg;
      debuf_modified_curvature_text.text = std::to_string(point.cost);
      
      unique_id++;
      
      points_marker_array.markers.push_back(debuf_modified_curvature_text); 
    }
    
    // // visualize debug goal point
    // visualization_msgs::Marker debug_goal_point;
    // debug_goal_point.lifetime = ros::Duration(0.2);
    // debug_goal_point.header = in_pose_ptr_->header;
    // debug_goal_point.ns = std::string("debug_goal_point_marker");
    // debug_goal_point.action = visualization_msgs::Marker::MODIFY;
    // debug_goal_point.pose.orientation.w = 1.0;
    // debug_goal_point.id = unique_id;
    // debug_goal_point.type = visualization_msgs::Marker::SPHERE_LIST;
    // debug_goal_point.scale.x = 0.9;
    // debug_goal_point.color.r = 0.0f;
    // debug_goal_point.color.g = 1.0f;
    // debug_goal_point.color.a = 1;
    // debug_goal_point.points.push_back(goal_point);
    // points_marker_array.markers.push_back(debug_goal_point);
    // unique_id++;
    
    // //text
    // size_t debug_reference_point_id = 0;
    // for (const auto& point: out_target_points)
    // {
    //   visualization_msgs::Marker debug_reference_point_text;
    //   debug_reference_point_text.lifetime = ros::Duration(0.2);
    //   debug_reference_point_text.header = in_pose_ptr_->header;
    //   debug_reference_point_text.ns = std::string("debug_reference_point_text");
    //   debug_reference_point_text.action = visualization_msgs::Marker::ADD;
    //   debug_reference_point_text.id = unique_id;
    //   debug_reference_point_text.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    //   debug_reference_point_text.scale.x = 1;
    //   debug_reference_point_text.scale.y = 0.1;
    //   debug_reference_point_text.scale.z = 0.4;

    //   // texts are green
    //   debug_reference_point_text.color.g = 1.0f;
    //   debug_reference_point_text.color.a = 1.0;
      
      
    //   geometry_msgs::Point relative_p;
    //   relative_p.y = 0.8;
    //   geometry_msgs::Pose pose;
    //   pose.position = point;
    //   pose.orientation.x = 0;
    //   pose.orientation.y = 0;
    //   pose.orientation.z = 0;
    //   pose.orientation.w = 1.0;
    //   tf::Transform inverse;
    //   tf::poseMsgToTF(pose, inverse);

    //   tf::Point p;
    //   pointMsgToTF(relative_p, p);
    //   tf::Point tf_p = inverse * p;
    //   geometry_msgs::Point tf_point_msg;
    //   pointTFToMsg(tf_p, tf_point_msg);
    //   debug_reference_point_text.pose.position = tf_point_msg;
    //   debug_reference_point_text.text = std::to_string(debug_reference_point_id);
    //   debug_reference_point_id ++;
      
    //   unique_id++;
      
    //   points_marker_array.markers.push_back(debug_reference_point_text); 
    // }
    
    // //center point text
    // size_t debug_global_point_id = 0;
    // for (const auto& point: local_center_points)
    // {
    //   visualization_msgs::Marker debug_center_point_text;
    //   debug_center_point_text.lifetime = ros::Duration(0.2);
    //   debug_center_point_text.header = in_pose_ptr_->header;
    //   debug_center_point_text.ns = std::string("debug_center_point_text");
    //   debug_center_point_text.action = visualization_msgs::Marker::ADD;
    //   debug_center_point_text.id = unique_id;
    //   debug_center_point_text.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    //   debug_center_point_text.scale.x = 1;
    //   debug_center_point_text.scale.y = 0.1;
    //   debug_center_point_text.scale.z = 0.4;

    //   // texts are green
    //   debug_center_point_text.color.g = 1.0f;
    //   debug_center_point_text.color.a = 1.0;
      
      
    //   geometry_msgs::Point relative_p;
    //   relative_p.y = 0.8;
    //   geometry_msgs::Pose pose;
    //   pose.position.x = point.tx ;
    //   pose.position.y = point.ty ;
    //   pose.position.z = in_waypoints_ptr_->waypoints.front().pose.pose.position.z;
    //   pose.orientation.x = 0;
    //   pose.orientation.y = 0;
    //   pose.orientation.z = 0;
    //   pose.orientation.w = 1.0;
    //   tf::Transform inverse;
    //   tf::poseMsgToTF(pose, inverse);

    //   tf::Point p;
    //   pointMsgToTF(relative_p, p);
    //   tf::Point tf_p = inverse * p;
    //   geometry_msgs::Point tf_point_msg;
    //   pointTFToMsg(tf_p, tf_point_msg);
    //   debug_center_point_text.pose.position = tf_point_msg;
    //   debug_center_point_text.text = std::to_string(point.cumulated_s).substr(0, 5);
    //   // debug_center_point_text.text += std::string(" ");
    //   // debug_center_point_text.text += std::to_string(point.tx);
    //   // debug_center_point_text.text += std::string(" ");
    //   // debug_center_point_text.text += std::to_string(point.ty);
      
    //   debug_global_point_id ++;
      
    //   unique_id++;
      
    //   points_marker_array.markers.push_back(debug_center_point_text); 
    // }

    
    visualization_msgs::Marker trajectory_marker;
    trajectory_marker.lifetime = ros::Duration(0.2);
    trajectory_marker.header = in_pose_ptr_->header;
    trajectory_marker.ns = std::string("trajectory_marker");
    trajectory_marker.action = visualization_msgs::Marker::MODIFY;
    trajectory_marker.pose.orientation.w = 1.0;
    trajectory_marker.id = unique_id;
    trajectory_marker.type = visualization_msgs::Marker::SPHERE_LIST;
    trajectory_marker.scale.x = 0.6;

    // Points are red
    trajectory_marker.color.r = 1.0f;
    trajectory_marker.color.a = 1;
    for (const auto& waypoint: out_trajectory.waypoints)
    {
      geometry_msgs::Point geometry_point;
      geometry_point.x = waypoint.pose.pose.position.x;
      geometry_point.y = waypoint.pose.pose.position.y;
      geometry_point.z = waypoint.pose.pose.position.z;
      trajectory_marker.points.push_back(geometry_point);
    }
    points_marker_array.markers.push_back(trajectory_marker);
    unique_id++;
    
    //text
    size_t debug_wp_id = 0;
    for (const auto& waypoint: out_trajectory.waypoints)
    {
      visualization_msgs::Marker trajectory_points_text;
      trajectory_points_text.lifetime = ros::Duration(0.2);
      trajectory_points_text.header = in_pose_ptr_->header;
      trajectory_points_text.ns = std::string("trajectory_points_text");
      trajectory_points_text.action = visualization_msgs::Marker::ADD;
      trajectory_points_text.id = unique_id;
      trajectory_points_text.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
      trajectory_points_text.scale.x = 1;
      trajectory_points_text.scale.y = 0.1;
      trajectory_points_text.scale.z = 0.4;

      // texts are green
      trajectory_points_text.color.g = 1.0f;
      trajectory_points_text.color.a = 1.0;
      
      double velocity_in_kmh = (waypoint.twist.twist.linear.x*60*60)/(1000);
      
      geometry_msgs::Point relative_p;
      relative_p.y = 0.8;
      geometry_msgs::Pose pose = waypoint.pose.pose;
      tf::Transform inverse;
      tf::poseMsgToTF(pose, inverse);

      tf::Point p;
      pointMsgToTF(relative_p, p);
      tf::Point tf_p = inverse * p;
      geometry_msgs::Point tf_point_msg;
      pointTFToMsg(tf_p, tf_point_msg);
      trajectory_points_text.pose.position = tf_point_msg;
      trajectory_points_text.text = std::to_string(debug_wp_id);
      trajectory_points_text.text += std::string(": ");
      
      trajectory_points_text.text += 
        std::to_string(velocity_in_kmh).substr(0,4);
        
      // trajectory_points_text.text += std::string(" ");
      // trajectory_points_text.text += std::to_string(pose.position.x);
      // trajectory_points_text.text += std::string(" ");
      // trajectory_points_text.text += std::to_string(pose.position.y);
      
      // trajectory_points_text.pose.orientation = waypoint.pose.pose.orientation;
      unique_id++;
      
      points_marker_array.markers.push_back(trajectory_points_text);
      
      debug_wp_id++;
    }
    
    int trajectory_count = 0;
    for(const auto& trajectory: out_debug_trajectories)
    {
      trajectory_count += 1;
      visualization_msgs::Marker trajectory_marker;
      trajectory_marker.lifetime = ros::Duration(0.2);
      trajectory_marker.header = in_pose_ptr_->header;
      trajectory_marker.ns = std::string("debug_trajectory_marker") ;
      trajectory_marker.action = visualization_msgs::Marker::MODIFY;
      trajectory_marker.pose.orientation.w = 1.0;
      trajectory_marker.id = unique_id;
      trajectory_marker.type = visualization_msgs::Marker::SPHERE_LIST;
      trajectory_marker.scale.x = 0.5;

      // Points are red
      trajectory_marker.color.r = 1.0f;
      trajectory_marker.color.a = 0.3;
      for (const auto& waypoint: trajectory.waypoints)
      {
        geometry_msgs::Point geometry_point;
        geometry_point.x = waypoint.pose.pose.position.x;
        geometry_point.y = waypoint.pose.pose.position.y;
        geometry_point.z = waypoint.pose.pose.position.z;
        trajectory_marker.points.push_back(geometry_point);
      }
      points_marker_array.markers.push_back(trajectory_marker);
      unique_id++; 
    }
    
    markers_pub_.publish(points_marker_array);
  }
}

void FrenetPlannerROS::loadVectormap()
{
  vectormap_load_ptr_->load();
}

// TODO: make this faster by kd-tree
Point FrenetPlannerROS::getNearestPoint(const geometry_msgs::PoseStamped& ego_pose)
{
  double min_dist = 99999;
  Point nearest_point;
  for(const auto& point: vectormap_load_ptr_->points_)
  {
    double dx = ego_pose.pose.position.x - point.tx;
    double dy = ego_pose.pose.position.y - point.ty;
    double dist = std::sqrt(std::pow(dx, 2)+std::pow(dy, 2));
    if(dist < min_dist)
    {
      min_dist = dist;
      nearest_point = point;
    }
  }
  return nearest_point;
}