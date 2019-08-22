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


#include <grid_map_ros/GridMapRosConverter.hpp>

#include <autoware_msgs/Lane.h>
#include <autoware_msgs/DetectedObjectArray.h>

#include "frenet_planner.h"
#include "vectormap_ros.h"
#include "vectormap_struct.h"

#include "calculate_center_line.h"

#include "frenet_planner_ros.h"

FrenetPlannerROS::FrenetPlannerROS()
  : nh_(), 
  private_nh_("~"),
  use_global_waypoints_as_center_line_(true),
  has_calculated_center_line_from_global_waypoints_(false)
{
  frenet_planner_ptr_.reset(new FrenetPlanner());
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
  
  tf2_buffer_ptr_.reset(new tf2_ros::Buffer());
  tf2_listner_ptr_.reset(new tf2_ros::TransformListener(*tf2_buffer_ptr_));
  
  
  optimized_waypoints_pub_ = nh_.advertise<autoware_msgs::Lane>("final_waypoints", 1, true);
  markers_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("frenet_planner_debug_markes", 1, true);
  final_waypoints_sub_ = nh_.subscribe("base_waypoints", 1, &FrenetPlannerROS::waypointsCallback, this);
  current_pose_sub_ = nh_.subscribe("/current_pose", 1, &FrenetPlannerROS::currentPoseCallback, this);
  current_velocity_sub_ = nh_.subscribe("/current_velocity", 1, &FrenetPlannerROS::currentVelocityCallback, this);
  objects_sub_ = nh_.subscribe("/detection/fake_perception/objects", 1, &FrenetPlannerROS::objectsCallback, this);
  // double timer_callback_dt = 0.05;
  double timer_callback_dt = 0.1;
  // double timer_callback_dt = 1.0;
  // double timer_callback_dt = 0.5;
  timer_ = nh_.createTimer(ros::Duration(timer_callback_dt), &FrenetPlannerROS::timerCallback, this);
  

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

void FrenetPlannerROS::objectsCallback(const autoware_msgs::DetectedObjectArray& msg)
{
  if(in_waypoints_ptr_)
  {
    if(msg.objects.size() == 0)
    {
      std::cerr << "ssize of objects is 0" << std::endl;
      return;
    }
    geometry_msgs::TransformStamped objects2map_tf;
    try
    {
        objects2map_tf = tf2_buffer_ptr_->lookupTransform(
          /*target*/  in_waypoints_ptr_->header.frame_id, 
          /*src*/ msg.header.frame_id,
          ros::Time(0));
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
      tf2::doTransform(current_object_pose, transformed_pose, objects2map_tf);
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
  
  if(in_pose_ptr_ && 
     in_twist_ptr_ && 
     in_waypoints_ptr_) 
  { 
    // 1. 現在日時を取得
    std::chrono::high_resolution_clock::time_point begin = std::chrono::high_resolution_clock::now();

    
  
    //TODO: refactor 
    std::vector<Point> local_center_points;
    if(use_global_waypoints_as_center_line_)
    {
      if(!has_calculated_center_line_from_global_waypoints_)
      {
        //run calculation oncen
        std::vector<Point> center_line_points;
        center_line_points
        = calculate_center_line_ptr_->calculateCenterLineFromGlobalWaypoints(
          in_waypoints_ptr_->waypoints);
        global_center_points_ptr_.reset(new std::vector<Point>(center_line_points));
        has_calculated_center_line_from_global_waypoints_ = true;
      }
      double min_dist = 99999;
      size_t closest_point_index = 0;
      for (size_t i = 0; i < global_center_points_ptr_->size(); i++)
      {
        double dx = global_center_points_ptr_->at(i).tx - in_pose_ptr_->pose.position.x;
        double dy = global_center_points_ptr_->at(i).ty - in_pose_ptr_->pose.position.y;
        double distance = std::sqrt(std::pow(dx, 2)+std::pow(dy,2));
        if(distance < min_dist)
        {
          min_dist = distance;
          closest_point_index = i;
        }
      }
      //TODO: think better way
      for(size_t i = closest_point_index; i< global_center_points_ptr_->size(); i++)
      {
        local_center_points.push_back(global_center_points_ptr_->at(i));
      }
    }
    else
    {
      Point nearest_lane_point = getNearestPoint(*in_pose_ptr_);
      local_center_points = nearest_lane_point.points;
    }
    
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
    
    
    //TODO: somehow improve interface
    autoware_msgs::Lane out_trajectory;
    std::vector<autoware_msgs::Lane> out_debug_trajectories;
    std::vector<geometry_msgs::Point> out_target_points;
    frenet_planner_ptr_->doPlan(*in_pose_ptr_, 
                                *in_twist_ptr_, 
                                local_center_points, 
                                local_reference_waypoints,
                                in_objects_ptr_,
                                wp2gridmap_tf_,
                                out_trajectory,
                                out_debug_trajectories,
                                out_target_points);
    // 3. 現在日時を再度取得
    std::chrono::high_resolution_clock::time_point end = std::chrono::high_resolution_clock::now();

    // 経過時間を取得
    std::chrono::nanoseconds elapsed_time = std::chrono::duration_cast<std::chrono::nanoseconds>(end - begin);
    std::cout << elapsed_time.count()/(1000.0*1000.0)<< " milli sec" << std::endl;
    // std::cerr << "output num wps" << out_trajectory.waypoints.size() << std::endl;
    std::cerr << "------------"  << std::endl;
    
    optimized_waypoints_pub_.publish(out_trajectory);
    
    
    
    //debug
    visualization_msgs::MarkerArray points_marker_array;
    int unique_id = 0;
    visualization_msgs::Marker reference_lane_points_marker;
    reference_lane_points_marker.lifetime = ros::Duration(0.2);
    reference_lane_points_marker.header = in_pose_ptr_->header;
    reference_lane_points_marker.ns = std::string("lane_points_marker");
    reference_lane_points_marker.action = visualization_msgs::Marker::MODIFY;
    reference_lane_points_marker.pose.orientation.w = 1.0;
    reference_lane_points_marker.id = unique_id;
    reference_lane_points_marker.type = visualization_msgs::Marker::SPHERE_LIST;
    reference_lane_points_marker.scale.x = 0.5;

    // Points are yellow
    reference_lane_points_marker.color.r = 1.0f;
    reference_lane_points_marker.color.g = 1.0f;
    reference_lane_points_marker.color.a = 1;
    for (const auto& next_point: local_center_points)
    {
      geometry_msgs::Point geometry_point;
      geometry_point.x = next_point.tx;
      geometry_point.y = next_point.ty;
      geometry_point.z = next_point.rz;
      reference_lane_points_marker.points.push_back(geometry_point);
    }
    points_marker_array.markers.push_back(reference_lane_points_marker);
    unique_id++;
    
    
    // visualize debug target point
    visualization_msgs::Marker debug_target_point;
    debug_target_point.lifetime = ros::Duration(0.2);
    debug_target_point.header = in_pose_ptr_->header;
    debug_target_point.ns = std::string("debug_target_point_marker");
    debug_target_point.action = visualization_msgs::Marker::MODIFY;
    debug_target_point.pose.orientation.w = 1.0;
    debug_target_point.id = unique_id;
    debug_target_point.type = visualization_msgs::Marker::SPHERE_LIST;
    debug_target_point.scale.x = 0.9;
    debug_target_point.color.r = 1.0f;
    debug_target_point.color.g = 1.0f;
    debug_target_point.color.a = 1;
    for(const auto& point: out_target_points)
    {
      debug_target_point.points.push_back(point);
    }
    points_marker_array.markers.push_back(debug_target_point);
    unique_id++;
    
    //text
    size_t debug_reference_point_id = 0;
    for (const auto& point: out_target_points)
    {
      visualization_msgs::Marker debug_reference_point_text;
      debug_reference_point_text.lifetime = ros::Duration(0.2);
      debug_reference_point_text.header = in_pose_ptr_->header;
      debug_reference_point_text.ns = std::string("debug_reference_point_text");
      debug_reference_point_text.action = visualization_msgs::Marker::ADD;
      debug_reference_point_text.id = unique_id;
      debug_reference_point_text.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
      debug_reference_point_text.scale.x = 1;
      debug_reference_point_text.scale.y = 0.1;
      debug_reference_point_text.scale.z = 0.4;

      // texts are green
      debug_reference_point_text.color.g = 1.0f;
      debug_reference_point_text.color.a = 1.0;
      
      
      geometry_msgs::Point relative_p;
      relative_p.y = 0.8;
      geometry_msgs::Pose pose;
      pose.position = point;
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
      debug_reference_point_text.pose.position = tf_point_msg;
      debug_reference_point_text.text = std::to_string(debug_reference_point_id);
      debug_reference_point_id ++;
      
      unique_id++;
      
      points_marker_array.markers.push_back(debug_reference_point_text); 
    }
    
    //center point text
    size_t debug_global_point_id = 0;
    for (const auto& point: local_center_points)
    {
      visualization_msgs::Marker debug_center_point_text;
      debug_center_point_text.lifetime = ros::Duration(0.2);
      debug_center_point_text.header = in_pose_ptr_->header;
      debug_center_point_text.ns = std::string("debug_center_point_text");
      debug_center_point_text.action = visualization_msgs::Marker::ADD;
      debug_center_point_text.id = unique_id;
      debug_center_point_text.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
      debug_center_point_text.scale.x = 1;
      debug_center_point_text.scale.y = 0.1;
      debug_center_point_text.scale.z = 0.4;

      // texts are green
      debug_center_point_text.color.g = 1.0f;
      debug_center_point_text.color.a = 1.0;
      
      
      geometry_msgs::Point relative_p;
      relative_p.y = 0.8;
      geometry_msgs::Pose pose;
      pose.position.x = point.tx ;
      pose.position.y = point.ty ;
      pose.position.z = in_waypoints_ptr_->waypoints.front().pose.pose.position.z;
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
      debug_center_point_text.pose.position = tf_point_msg;
      debug_center_point_text.text = std::to_string(point.cumulated_s).substr(0, 5);
      // debug_center_point_text.text += std::to_string(point.tx);
      // debug_center_point_text.text += std::string(" ");
      // debug_center_point_text.text += std::to_string(point.ty);
      
      debug_global_point_id ++;
      
      unique_id++;
      
      points_marker_array.markers.push_back(debug_center_point_text); 
    }

    
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
    
    //arrow
    for (const auto& waypoint: out_trajectory.waypoints)
    {
      visualization_msgs::Marker trajectory_arrow_marker;
      trajectory_arrow_marker.lifetime = ros::Duration(0.2);
      trajectory_arrow_marker.header = in_pose_ptr_->header;
      trajectory_arrow_marker.ns = std::string("trajectory_arrow_marker");
      trajectory_arrow_marker.action = visualization_msgs::Marker::ADD;
      trajectory_arrow_marker.id = unique_id;
      trajectory_arrow_marker.type = visualization_msgs::Marker::ARROW;
      trajectory_arrow_marker.scale.x = 1;
      trajectory_arrow_marker.scale.y = 0.1;
      trajectory_arrow_marker.scale.z = 0.1;

      // Arrows are blue
      trajectory_arrow_marker.color.b = 1.0f;
      trajectory_arrow_marker.color.a = 1.0;
      trajectory_arrow_marker.pose.position = waypoint.pose.pose.position;
      trajectory_arrow_marker.pose.orientation = waypoint.pose.pose.orientation;
      unique_id++;
      
      points_marker_array.markers.push_back(trajectory_arrow_marker);
    }
    
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