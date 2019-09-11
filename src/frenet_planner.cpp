#include <autoware_msgs/LaneArray.h>
#include <autoware_msgs/DetectedObjectArray.h>

#include <tf2/utils.h>
#include <tf/transform_datatypes.h>

//headers in Eigen
#include <Eigen/Dense>

#include "frenet_planner.h"
#include "vectormap_struct.h"

#include <numeric>



//TODO: make namespace/file for utility method
//TODO: better naming 
double calculate2DDistace(const geometry_msgs::Point& point1,
                          const geometry_msgs::Point& point2)
{
  double dx = point1.x - point2.x;
  double dy = point1.y - point2.y;
  double distance = std::sqrt(std::pow(dx, 2)+std::pow(dy,2));
  return distance;
}

// ref: http://www.mech.tohoku-gakuin.ac.jp/rde/contents/course/robotics/coordtrans.html
// (pu, pv): retative, (px, py): absolute, (ox, oy): origin
// (pu, pv) = rot^-1 * {(px, py) - (ox, oy)}
geometry_msgs::Point transformToRelativeCoordinate2D(const geometry_msgs::Point &point,
                                                     const geometry_msgs::Pose &origin)
{
  geometry_msgs::Point res;

  // translation
  geometry_msgs::Point trans_p;
  trans_p.x = point.x - origin.position.x;
  trans_p.y = point.y - origin.position.y;
  

  // rotation (use inverse matrix of rotation)
  double yaw = tf2::getYaw(origin.orientation);
  res.x = (cos(yaw) * trans_p.x) + (sin(yaw) * trans_p.y);
  res.y = ((-1) * sin(yaw) * trans_p.x) + (cos(yaw) * trans_p.y);
  res.z = origin.position.z;

  return res;
}


//does not consider z axis information
//does not consider time axis motion of ego vehicle
FrenetPlanner::FrenetPlanner(
  double initial_velocity_ms,
  double velocity_ms_before_obstalcle,
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
  double converge_distance_per_ms_for_stop,
  double linear_velocity):
initial_velocity_ms_(initial_velocity_ms),
velcity_ms_before_obstalcle_(velocity_ms_before_obstalcle),
distance_before_obstalcle_(distance_before_obstalcle),
obstacle_radius_from_center_point_(obstacle_radius_from_center_point),
min_lateral_referencing_offset_for_avoidance_(min_lateral_referencing_offset_for_avoidance),
max_lateral_referencing_offset_for_avoidance_(max_lateral_referencing_offset_for_avoidance),
diff_waypoints_cost_coef_(diff_waypoints_cost_coef),
diff_last_waypoint_cost_coef_(diff_last_waypoint_cost_coef),
jerk_cost_coef_(jerk_cost_coef),
required_time_cost_coef_(required_time_cost_coef),
comfort_acceleration_cost_coef_(comfort_acceleration_cost_coef),
lookahead_distance_per_ms_for_reference_point_(lookahead_distance_per_ms_for_reference_point),
minimum_lookahead_distance_for_reference_point_(20.0),
lookahead_distance_for_reference_point_(minimum_lookahead_distance_for_reference_point_),
converge_distance_per_ms_for_stop_(converge_distance_per_ms_for_stop),
radius_from_reference_point_for_valid_trajectory_(10.0),
dt_for_sampling_points_(0.5),
linear_velocity_(linear_velocity)
{
}

FrenetPlanner::~FrenetPlanner()
{
}


void FrenetPlanner::doPlan(const geometry_msgs::PoseStamped& in_current_pose,
              const geometry_msgs::TwistStamped& in_current_twist,
              const std::vector<Point>& in_nearest_lane_points,
              const std::vector<autoware_msgs::Waypoint>& in_reference_waypoints,
              const std::unique_ptr<autoware_msgs::DetectedObjectArray>& in_objects_ptr,
              autoware_msgs::Lane& out_trajectory,
              std::vector<autoware_msgs::Lane>& out_debug_trajectories,
              std::vector<geometry_msgs::Point>& out_reference_points)
{
  std::vector<autoware_msgs::Waypoint> entire_path;
  generateEntirePath(in_current_pose,
                     in_nearest_lane_points,
                     in_reference_waypoints,
                     in_objects_ptr,
                     entire_path,
                     out_debug_trajectories,
                     out_reference_points);
  out_trajectory.waypoints = entire_path;
  // previous_best_path_.reset(new std::vector<autoware_msgs::Waypoint>(entire_path));
 
}

//TODO: better naming
bool FrenetPlanner::generateEntirePath(
  const geometry_msgs::PoseStamped& current_pose,
  const std::vector<Point>& lane_points,
  const std::vector<autoware_msgs::Waypoint>& reference_waypoints,
  const std::unique_ptr<autoware_msgs::DetectedObjectArray>& in_objects_ptr,
  std::vector<autoware_msgs::Waypoint>& entire_path,
  std::vector<autoware_msgs::Lane>& out_debug_trajectories,
  std::vector<geometry_msgs::Point>& out_reference_points)
{
  
  double frenet_s_position, frenet_d_position;
  convertCartesianPosition2FrenetPosition(current_pose.pose.position,
                                          lane_points,
                                          frenet_s_position,
                                          frenet_d_position);
  FrenetPoint origin_point;
  Point nearest_point;
  getNearestPoint(current_pose.pose.position,
                    lane_points,
                    nearest_point);
  origin_point.d_state(0) = 0;
  origin_point.s_state(0) = nearest_point.cumulated_s;
  double delta_s = 10;
  double number_of_path_layer = 2;
  //TODO: better naming
  geometry_msgs::Pose origin_pose = current_pose.pose;
  std::vector<Trajectory> trajectories;
  FrenetPoint target_point;
  target_point.d_state(0) = 0;
  target_point.s_state(0) = (origin_point.s_state(0) + delta_s);
  ReferencePoint reference_point;
  reference_point.frenet_point = target_point;
  reference_point.lateral_max_offset = 1.0;
  // reference_point.lateral_sampling_resolution = 0.25;
  reference_point.lateral_sampling_resolution = 0.25;
  reference_point.longitudinal_max_offset = 0.0;
  reference_point.longitudinal_sampling_resolution = 1.5;
  
  
  
  drawTrajectories(origin_pose,
                    origin_point,
                    reference_point,
                    lane_points,
                    reference_waypoints,
                    trajectories,
                    out_debug_trajectories);
  for(double current_target_path_length = (delta_s*number_of_path_layer); 
             current_target_path_length <= delta_s*number_of_path_layer;
             current_target_path_length += delta_s)
  {
    const std::vector<Trajectory> dc_trajectories = trajectories;
    trajectories.clear();
    for (const auto& dc_trajectory: dc_trajectories)
    {
      geometry_msgs::Pose origin_pose = dc_trajectory.trajectory_points.waypoints.back().pose.pose;
      FrenetPoint origin_point = dc_trajectory.frenet_trajectory_points.back();
      FrenetPoint target_point;
      target_point.d_state(0) = origin_point.d_state(0);
      target_point.s_state(0) = (origin_point.s_state(0) + delta_s);
      ReferencePoint reference_point;
      reference_point.frenet_point = target_point;
      reference_point.lateral_max_offset = 2.0;
      reference_point.lateral_sampling_resolution = 0.25;
      reference_point.longitudinal_max_offset = 0.0;
      reference_point.longitudinal_sampling_resolution = 1.5;
      std::vector<Trajectory> child_trajectories;
      drawTrajectories(origin_pose,
                    origin_point,
                    reference_point,
                    lane_points,
                    reference_waypoints,
                    child_trajectories,
                    out_debug_trajectories);
                    
      for(auto& child_trajectory: child_trajectories)
      {
        Trajectory new_trajectory = dc_trajectory;
        new_trajectory.trajectory_points.waypoints.insert
                    (new_trajectory.trajectory_points.waypoints.end(),
                     child_trajectory.trajectory_points.waypoints.begin(),
                     child_trajectory.trajectory_points.waypoints.end());
        new_trajectory.calculated_trajectory_points.insert
                    (new_trajectory.calculated_trajectory_points.end(),
                     child_trajectory.calculated_trajectory_points.begin(),
                     child_trajectory.calculated_trajectory_points.end());
        new_trajectory.frenet_trajectory_points.insert
                    (new_trajectory.frenet_trajectory_points.end(),
                     child_trajectory.frenet_trajectory_points.begin(),
                     child_trajectory.frenet_trajectory_points.end());
        //TODO: conversion error from frenet to cartesion to frenet
        // std::cerr<<"input origin  "<<origin_point.d_state(0)<<std::endl;
        // std::cerr<<"dc "<<dc_trajectory.frenet_trajectory_points.back().d_state(0)<<std::endl;
        // std::cerr << "sexod " << child_trajectory.frenet_trajectory_points.front().d_state(0) << std::endl;
        trajectories.push_back(new_trajectory);
      }
    }
  }
  
  
  ReferencePoint final_reference_point;
  final_reference_point.frenet_point.d_state(0) = 0;
  final_reference_point.frenet_point.s_state(0) = delta_s*number_of_path_layer + origin_point.s_state(0);
  final_reference_point.lateral_max_offset = 2.0;
  final_reference_point.lateral_sampling_resolution = 0.25;
  final_reference_point.longitudinal_max_offset = 0.0;
  final_reference_point.longitudinal_sampling_resolution = 1.5;
  std::unique_ptr<ReferencePoint> kept_reference_point;
  kept_reference_point.reset(new ReferencePoint(final_reference_point));  
  std::unique_ptr<Trajectory> kept_best_trajectory;
  selectBestTrajectory(trajectories,
                       in_objects_ptr,
                       reference_waypoints,
                       kept_reference_point,
                       kept_best_trajectory);
  // origin_point = kept_best_trajectory->frenet_trajectory_points.back();
  // origin_pose = kept_best_trajectory->trajectory_points.waypoints.back().pose.pose;
  out_reference_points.push_back(origin_pose.position);
  entire_path = kept_best_trajectory->trajectory_points.waypoints;
  // std::cerr << "traf size " << trajectories.size() << std::endl;
  // std::cerr << "num  wp " << entire_path.size() << std::endl;
  // for(const auto& wp: entire_path)
  // {
  //   std::cerr << "wpx " << wp.pose.pose.position.x << std::endl;
  //   std::cerr << "wpx " << wp.pose.pose.position.y << std::endl;
  // }
  // entire_path.insert
  //             (entire_path.end(),
  //               kept_best_trajectory->trajectory_points.waypoints.begin(),
                // kept_best_trajectory->trajectory_points.waypoints.end());
}

//TODO: draw trajectories based on reference_point parameters
bool FrenetPlanner::drawTrajectories(
              const geometry_msgs::Pose& origin_pose,
              const FrenetPoint& frenet_current_point,
              const ReferencePoint& reference_point,
              const std::vector<Point>& lane_points,
              const std::vector<autoware_msgs::Waypoint>& reference_waypoints,
              std::vector<Trajectory>& trajectories,
              std::vector<autoware_msgs::Lane>& out_debug_trajectories)
{
  // std::cerr << "lateral offset " << reference_point.lateral_max_offset << std::endl;
  // std::cerr << "lateral samp " << reference_point.lateral_sampling_resolution << std::endl;
  for(double lateral_offset = -1*reference_point.lateral_max_offset; 
      lateral_offset<= reference_point.lateral_max_offset; 
      lateral_offset+=reference_point.lateral_sampling_resolution)
  {
    for(double longitudinal_offset = -1*reference_point.longitudinal_max_offset; 
        longitudinal_offset<= reference_point.longitudinal_max_offset; 
        longitudinal_offset+=reference_point.longitudinal_sampling_resolution)
    {
      FrenetPoint frenet_target_point;
      frenet_target_point.d_state = reference_point.frenet_point.d_state;
      frenet_target_point.s_state = reference_point.frenet_point.s_state;
      frenet_target_point.d_state(0) += lateral_offset;
      frenet_target_point.s_state(0) += longitudinal_offset;
      Trajectory trajectory;
      if(generateTrajectory(
          origin_pose,
          lane_points,
          reference_waypoints,
          frenet_current_point,
          frenet_target_point,
          200,
          dt_for_sampling_points_,
          trajectory))
      {
        trajectories.push_back(trajectory);
      }
      out_debug_trajectories.push_back(trajectory.trajectory_points);
    }
  }
  if(trajectories.size()==0)
  {
    std::cerr << "ERROR: no trajectory generated in drawTrajectories; please adjust jerk threshold"  << std::endl;
  }
}


//TODO: include current_pose and calcualate tan(delta_theta)
bool FrenetPlanner::generateTrajectory(
    const geometry_msgs::Pose& ego_pose,
    const std::vector<Point>& lane_points,
    const std::vector<autoware_msgs::Waypoint>& reference_waypoints,
    const FrenetPoint& origin_frenet_point,
    const FrenetPoint& reference_frenet_point,
    const double time_horizon,
    const double dt_for_sampling_points,
    Trajectory& trajectory)
{
  
  Point nearest_lane_point;
  getNearestPoint(ego_pose.position, lane_points, nearest_lane_point);
  double yaw = tf::getYaw(ego_pose.orientation);
  double lane_yaw = nearest_lane_point.rz;
  double delta_yaw = yaw - lane_yaw;
  // std::cerr << "delta yaw " << delta_yaw << std::endl;
  // std::cerr << "tan delta_yaw " << std::tan(delta_yaw) << std::endl;
  double origin_s = origin_frenet_point.s_state(0);
  double target_s = reference_frenet_point.s_state(0);
  double delta_s = target_s - origin_s;
  Eigen::Matrix3d a; 
  a << std::pow(delta_s, 3), std::pow(delta_s, 2), delta_s,
                          0,                    0,       1,
          3*delta_s*delta_s,            2*delta_s,       1;
  double target_d = reference_frenet_point.d_state(0);
  double origin_d = origin_frenet_point.d_state(0);
  Eigen::Vector3d b;
  b << target_d - origin_d, std::tan(delta_yaw), 0;
  Eigen::Vector3d x = a.inverse()*b;
  
  const size_t num_sample = 10;
  for(size_t i = 1; i <= num_sample; i++)
  {
    double calculated_s = i*delta_s/static_cast<double>(num_sample) + origin_s;
    double tmp_delta_s = calculated_s - origin_s;
    double calculated_d = x(0)*std::pow(tmp_delta_s, 3) + 
                          x(1)*std::pow(tmp_delta_s, 2) +
                          x(2)*tmp_delta_s +
                          origin_d;
    FrenetPoint calculated_frenet_point;
    calculated_frenet_point.s_state(0) = calculated_s;
    calculated_frenet_point.s_state(1) = linear_velocity_;
    calculated_frenet_point.s_state(2) = 0;
    calculated_frenet_point.s_state(3) = 0;
    calculated_frenet_point.d_state(0) = calculated_d;
    calculated_frenet_point.d_state(1) = 0;
    calculated_frenet_point.d_state(2) = 0;
    calculated_frenet_point.d_state(3) = 0;
    trajectory.frenet_trajectory_points.push_back(calculated_frenet_point);
    
    autoware_msgs::Waypoint waypoint;
    calculateWaypoint(lane_points, 
                      calculated_frenet_point,
                      waypoint);
    waypoint.pose.pose.position.z = reference_waypoints.front().pose.pose.position.z;
    trajectory.trajectory_points.waypoints.push_back(waypoint);
    
    TrajecotoryPoint trajectory_point;
    calculateTrajectoryPoint(lane_points, 
                             calculated_frenet_point,
                             trajectory_point);
    trajectory.calculated_trajectory_points.push_back(trajectory_point);
  }
  trajectory.required_time = time_horizon;
  return true;
}


bool FrenetPlanner::calculateWaypoint(
                           const std::vector<Point>& lane_points, 
                           const FrenetPoint& frenet_point,
                           autoware_msgs::Waypoint& waypoint)
{
  // add conversion script for low velocity
  // std::cerr << "-------" << std::endl;
  double s_position = frenet_point.s_state(0);
  double s_velocity = frenet_point.s_state(1);
  double s_acceleration = frenet_point.s_state(2);
  double d_position = frenet_point.d_state(0);
  double d_velocity = frenet_point.d_state(1);
  double d_acceleration = frenet_point.d_state(2);
  double min_abs_delta = 100000;
  double nearest_lane_point_delta_s;
  double nearest_lane_point_yaw;
  double nearest_lane_point_curvature;
  double nearest_lane_point_curvature_dot;
  geometry_msgs::Point nearest_lane_point;
  for (const auto& point: lane_points)
  {
    double delta_s = s_position - point.cumulated_s;
    if(std::abs(delta_s) < min_abs_delta)
    {
      min_abs_delta = std::abs(delta_s);
      nearest_lane_point_delta_s = delta_s;
      nearest_lane_point_yaw = point.rz;
      nearest_lane_point_curvature = point.curvature;
      nearest_lane_point_curvature_dot = point.curvature_dot;
      
      nearest_lane_point.x = point.tx;
      nearest_lane_point.y = point.ty;
      
    }
  }
  
  double velocity = std::sqrt(std::pow(1 - nearest_lane_point_curvature*d_position, 2)*
                              std::pow(s_velocity, 2) + 
                              std::pow(d_velocity,2));
  // double delta_yaw = nearest_lane_point_yaw - current_yaw;
  double d_dash = d_velocity/s_velocity;
  double d_double_dash = (1/(s_velocity*s_velocity))*(d_acceleration - s_acceleration*d_dash);
  double delta_yaw = std::atan(d_dash/(1 - nearest_lane_point_curvature * d_position));
  double waypoint_yaw = nearest_lane_point_yaw - delta_yaw;
  double waypoint_curvature = (std::pow(std::cos(delta_yaw),3)/
                               std::pow((1-nearest_lane_point_curvature*d_position),2))*
                              (d_double_dash +
                               (nearest_lane_point_curvature_dot*d_position +
                                nearest_lane_point_curvature*d_velocity)*
                                std::tan(delta_yaw)+
                               ((1 - nearest_lane_point_curvature*d_position)/
                               (std::pow(std::cos(delta_yaw),2)))*nearest_lane_point_curvature);
  double acceleration = s_acceleration*((1 - nearest_lane_point_curvature*d_position)/std::cos(delta_yaw))+
                        s_velocity*s_velocity/std::cos(delta_yaw)*
                        ((1 - nearest_lane_point_curvature*d_position)*std::tan(delta_yaw)*
                        (waypoint_curvature*((1 - nearest_lane_point_curvature*d_position)/std::cos(delta_yaw))-
                        nearest_lane_point_curvature) -
                        -1*(nearest_lane_point_curvature_dot*d_position+
                             nearest_lane_point_curvature*d_velocity));
  
  // std::cerr << "wp velocity " << velocity << std::endl;
  // std::cerr << "wp acceleration " << acceleration << std::endl;
  // std::cerr << "wp yaw " << waypoint_yaw << std::endl;
  // std::cerr << "wp curvature " << waypoint_curvature << std::endl;
  waypoint.pose.pose.orientation = tf::createQuaternionMsgFromYaw(waypoint_yaw);
  waypoint.twist.twist.linear.x = velocity;
  
  // double d_double_dash = -1*((nearest_lane_point_curvature_dot*d_position+
  //                            nearest_lane_point_curvature*d_velocity)*std::tan(delta_yaw))+
  //                            ((1 - nearest_lane_point_curvature*d_position)/std::cos(delta_yaw)*std::cos(delta_yaw))*
  //                            (nearest_lane_point_curvature*((1 - nearest_lane_point_curvature*d_position)/std::cos(delta_yaw))-
  //                            nearest_lane_point_curvature);
                  
  // double d_velocit
                              
                              
  // std::cerr << "nearest lane yaw " << nearest_lane_point_yaw << std::endl;
  // std::cerr << "current yaw " << current_yaw << std::endl;
  geometry_msgs::Point waypoint_position;
  convertFrenetPosition2CartesianPosition(s_position,
                                          d_position,
                                          nearest_lane_point,
                                          nearest_lane_point_delta_s,
                                          nearest_lane_point_yaw,
                                          waypoint_position);
  waypoint.pose.pose.position = waypoint_position;

  return true;
}

bool FrenetPlanner::calculateTrajectoryPoint(
                           const std::vector<Point>& lane_points, 
                           const FrenetPoint& frenet_point,
                           TrajecotoryPoint& trajectory_point)
{
  // add conversion script for low velocity
  // std::cerr << "-------" << std::endl;
  double s_position = frenet_point.s_state(0);
  double s_velocity = frenet_point.s_state(1);
  double s_acceleration = frenet_point.s_state(2);
  double d_position = frenet_point.d_state(0);
  double d_velocity = frenet_point.d_state(1);
  double d_acceleration = frenet_point.d_state(2);
  double min_abs_delta = 100000;
  double nearest_lane_point_delta_s;
  double nearest_lane_point_yaw;
  double nearest_lane_point_curvature;
  double nearest_lane_point_curvature_dot;
  geometry_msgs::Point nearest_lane_point;
  for (const auto& point: lane_points)
  {
    double delta_s = s_position - point.cumulated_s;
    if(std::abs(delta_s) < min_abs_delta)
    {
      min_abs_delta = std::abs(delta_s);
      nearest_lane_point_delta_s = delta_s;
      nearest_lane_point_yaw = point.rz;
      nearest_lane_point_curvature = point.curvature;
      nearest_lane_point_curvature_dot = point.curvature_dot;
      
      nearest_lane_point.x = point.tx;
      nearest_lane_point.y = point.ty;
      
    }
  }
  
  double velocity = std::sqrt(std::pow(1 - nearest_lane_point_curvature*d_position, 2)*
                              std::pow(s_velocity, 2) + 
                              std::pow(d_velocity,2));
  // std::cerr << "n-curv " << nearest_lane_point_curvature 
  //           << "s_velo " << s_velocity
  //           << "linear v "<< velocity
  //           << std::endl;
  // double delta_yaw = nearest_lane_point_yaw - current_yaw;
  double d_dash = d_velocity/s_velocity;
  double d_double_dash = (1/(s_velocity*s_velocity))*(d_acceleration - s_acceleration*d_dash);
  double delta_yaw = std::atan(d_dash/(1 - nearest_lane_point_curvature * d_position));
  double waypoint_yaw = nearest_lane_point_yaw - delta_yaw;
  double waypoint_curvature = (std::pow(std::cos(delta_yaw),3)/
                               std::pow((1-nearest_lane_point_curvature*d_position),2))*
                              (d_double_dash +
                               (nearest_lane_point_curvature_dot*d_position +
                                nearest_lane_point_curvature*d_velocity)*
                                std::tan(delta_yaw)+
                               ((1 - nearest_lane_point_curvature*d_position)/
                               (std::pow(std::cos(delta_yaw),2)))*nearest_lane_point_curvature);
  double acceleration = s_acceleration*((1 - nearest_lane_point_curvature*d_position)/std::cos(delta_yaw))+
                        s_velocity*s_velocity/std::cos(delta_yaw)*
                        ((1 - nearest_lane_point_curvature*d_position)*std::tan(delta_yaw)*
                        (waypoint_curvature*((1 - nearest_lane_point_curvature*d_position)/std::cos(delta_yaw))-
                        nearest_lane_point_curvature) -
                        -1*(nearest_lane_point_curvature_dot*d_position+
                             nearest_lane_point_curvature*d_velocity));
  
  geometry_msgs::Point trajectory_cartesian_point;
  convertFrenetPosition2CartesianPosition(s_position,
                                          d_position,
                                          nearest_lane_point,
                                          nearest_lane_point_delta_s,
                                          nearest_lane_point_yaw,
                                          trajectory_cartesian_point);
  trajectory_point.x = trajectory_cartesian_point.x;
  trajectory_point.x = trajectory_cartesian_point.y;
  trajectory_point.yaw = waypoint_yaw;
  trajectory_point.curvature = waypoint_curvature;
  trajectory_point.velocity = velocity;
  trajectory_point.accerelation = acceleration;

  return true;
}


bool FrenetPlanner::convertFrenetPosition2CartesianPosition(
                           const double frenet_s,
                           const double frenet_d,
                           const geometry_msgs::Point& nearest_lane_cartesian_point,
                           const double delta_s,
                           const double delta_yaw,
                           geometry_msgs::Point& waypoint_position)
{
  waypoint_position.x = nearest_lane_cartesian_point.x + delta_s*std::cos(delta_yaw);
  waypoint_position.y = nearest_lane_cartesian_point.y + delta_s*std::sin(delta_yaw);
  waypoint_position.x += frenet_d*std::cos(delta_yaw-M_PI/2);
  waypoint_position.y += frenet_d*std::sin(delta_yaw-M_PI/2);

  return true;
  // std::cerr <<"cumulated s "<< point.cumulated_s << std::endl;
}

bool FrenetPlanner::convertCartesianPosition2FrenetPosition(
        const geometry_msgs::Point& cartesian_point,
        const std::vector<Point> lane_points,        
        double& frenet_s_position,
        double& frenet_d_position)
{
  //TODO: redundant
  Point nearest_point, second_nearest_point;
  getNearestPoints(cartesian_point,
                    lane_points,
                    nearest_point,
                    second_nearest_point);
  double x1 = nearest_point.tx;
  double y1 = nearest_point.ty;
  double x2 = second_nearest_point.tx;
  double y2 = second_nearest_point.ty;
  
  // ax + by + c = 0 which pass point1 and point2
  double line_a = (y2 - y1)/(x2 - x1);
  double line_b = -1;
  double line_c = y1 - (x1*(y2 - y1))/(x2 - x1);
  
  // distance between point3 and line
  double x3 = cartesian_point.x;
  double y3 = cartesian_point.y;
  double current_d_position = std::abs(line_a*x3 + line_b*y3 + line_c)/std::sqrt(line_a*line_a+line_b*line_b);
  
  // check current_d is positive or negative by using cross product
  double tmp_det_z;
  tmp_det_z = (x2 - x1)*(y3 - y1) - (y2 - y1)*(x3 - x1);
  if (tmp_det_z > 0)
  {
    current_d_position*= -1;
  }
  
  
  
  
  // line equation perpendicular to ax + by + c = 0 && pass point3
  double p_line_a = line_b;
  double p_line_b = -1* line_a;
  double p_line_c = line_a*y3 - line_b*x3; 
  
  // again distance between point1 and p_line
  double current_s_position = std::abs(p_line_a*x1 + p_line_b*y1+ p_line_c)/std::sqrt(p_line_a*p_line_a+p_line_b*p_line_b);
  
  // check if current_s is positive or negative
  // calculate another point in p_line
  double x_a = x3 + 1;
  double y_a = -1* p_line_a*x_a/p_line_b -1*p_line_c/p_line_b;
  tmp_det_z = (x3 - x_a)*(y1 - y_a) - (y3 - y_a)*(x1 - x_a);
  if (tmp_det_z < 0)
  {
    current_s_position*= -1;
  }
  
  
  current_s_position += nearest_point.cumulated_s;

  // Eigen::Vecto
  frenet_d_position = current_d_position;
  frenet_s_position = current_s_position;
}
    
    
// TODO: redundant
void FrenetPlanner::getNearestPoints(const geometry_msgs::Point& point,
                                    const std::vector<Point>& nearest_lane_points,
                                    Point& nearest_point,
                                    Point& second_nearest_point)
{
  double min_dist = 99999;
  for(const auto& sub_point: nearest_lane_points)
  {
    double dx = point.x - sub_point.tx;
    double dy = point.y - sub_point.ty;
    double dist = std::sqrt(std::pow(dx, 2)+std::pow(dy, 2));
    if(dist < min_dist)
    {
      min_dist = dist;
      nearest_point = sub_point;
    }
  }
  double second_min_dist = 99999;
  bool is_initialized = false;
  for(size_t i = 0; i < nearest_lane_points.size(); i++)
  {
    double dx = point.x - nearest_lane_points[i].tx;
    double dy = point.y - nearest_lane_points[i].ty;
    double dist = std::sqrt(std::pow(dx, 2)+std::pow(dy, 2));
    if(dist <= min_dist && (i+1 < nearest_lane_points.size()))
    {
      second_nearest_point = nearest_lane_points[i+1];
      is_initialized = true;
    }
  }
  if (!is_initialized)
  {
    std::cerr << "second is not initialized"  << std::endl;
  }
  
  
  double threshold = 3;
  if(min_dist > threshold)
  {
    std::cerr << "error: target is too far; might not be valid goal" << std::endl;
  }
}

void FrenetPlanner::getNearestPoint(const geometry_msgs::Point& compare_point,
                                    const std::vector<Point>& lane_points,
                                    Point& nearest_point)
{
  double min_dist = 99999;
  for(const auto& sub_point: lane_points)
  {
    double dx = compare_point.x - sub_point.tx;
    double dy = compare_point.y - sub_point.ty;
    double dist = std::sqrt(std::pow(dx, 2)+std::pow(dy, 2));
    if(dist < min_dist)
    {
      min_dist = dist;
      nearest_point = sub_point;
    }
  }
}

// TODO: redundant 
// TODO: make it faster
void FrenetPlanner::getNearestWaypoints(const geometry_msgs::Pose& point,
                                    const autoware_msgs::Lane& waypoints,
                                    autoware_msgs::Waypoint& nearest_waypoint,
                                    autoware_msgs::Waypoint& second_nearest_waypoint)
{
  double min_dist = 99999;
  size_t min_waypoint_index = 0;
  for(size_t i = 0 ; i < waypoints.waypoints.size(); i++)
  {
    double dx = point.position.x - waypoints.waypoints[i].pose.pose.position.x;
    double dy = point.position.y - waypoints.waypoints[i].pose.pose.position.y;
    double dist = std::sqrt(std::pow(dx, 2)+std::pow(dy, 2));
    if(dist < min_dist)
    {
      min_dist = dist;
      min_waypoint_index = i;
      nearest_waypoint = waypoints.waypoints[i];
    }
  }
  
  size_t second_min_waypoint_index = 0;
  if(min_waypoint_index == (waypoints.waypoints.size()-1))
  {
    second_min_waypoint_index = min_waypoint_index - 1; 
  }
  else
  {
    second_min_waypoint_index = min_waypoint_index + 1; 
  }
  second_nearest_waypoint = waypoints.waypoints[second_min_waypoint_index]; 
}

// TODO: redundant 
// TODO: make it faster
void FrenetPlanner::getNearestWaypoint(const geometry_msgs::Point& point,
                                    const std::vector<autoware_msgs::Waypoint>& waypoints,
                                    autoware_msgs::Waypoint& nearest_waypoint)
{
  double min_dist = 99999;
  for(size_t i = 0 ; i < waypoints.size(); i++)
  {
    double dx = point.x - waypoints[i].pose.pose.position.x;
    double dy = point.y - waypoints[i].pose.pose.position.y;
    double dist = std::sqrt(std::pow(dx, 2)+std::pow(dy, 2));
    if(dist < min_dist)
    {
      min_dist = dist;
      nearest_waypoint = waypoints[i];
    }
  } 
}

//TODO: make method for redundant part
void FrenetPlanner::getNearestWaypointIndex(const geometry_msgs::Point& point,
                                    const std::vector<autoware_msgs::Waypoint>& waypoints,
                                    size_t& nearest_waypoint_index)
{
  double min_dist = 99999;
  for(size_t i = 0 ; i < waypoints.size(); i++)
  {
    double dx = point.x - waypoints[i].pose.pose.position.x;
    double dy = point.y - waypoints[i].pose.pose.position.y;
    double dist = std::sqrt(std::pow(dx, 2)+std::pow(dy, 2));
    if(dist < min_dist)
    {
      min_dist = dist;
      nearest_waypoint_index = i;
    }
  } 
}


bool FrenetPlanner::selectBestTrajectory(
      const std::vector<Trajectory>& trajectories,
      const std::unique_ptr<autoware_msgs::DetectedObjectArray>& objects_ptr,
      const std::vector<autoware_msgs::Waypoint>& reference_waypoints, 
      std::unique_ptr<ReferencePoint>& kept_reference_point,
      std::unique_ptr<Trajectory>& kept_best_trajectory)
{
  
  std::vector<double> ref_waypoints_costs;
  std::vector<double> ref_last_waypoints_costs;
  std::vector<double> debug_last_waypoints_costs_s;
  std::vector<double> debug_last_waypoints_costs_d;
  std::vector<double> debug_last_waypoints_actual_s;
  std::vector<double> debug_last_waypoints_actual_d;
  std::vector<double> jerk_costs;
  std::vector<double> required_time_costs;
  std::vector<double> comfort_accerelation_costs;
  std::vector<double> debug_max_s_jerk;
  std::vector<double> debug_max_d_jerk;
  double sum_ref_waypoints_costs = 0;
  double sum_last_waypoints_costs = 0;
  double sum_jerk_costs = 0;
  double sum_required_time_costs = 0;
  double sum_comfort_accerelation_costs = 0;
  for(const auto& trajectory: trajectories)
  {
    
    //calculate terminal cost
    FrenetPoint frenet_point_at_time_horizon = trajectory.frenet_trajectory_points.back();
    // std::cerr << "traj frenet at time horizon " << 
    //         frenet_point_at_time_horizon.d_state(0)<< " " <<
    //         frenet_point_at_time_horizon.s_state(0) << std::endl;
    // std::cerr << "reference frenet at time horizon " << 
    //         kept_reference_point->frenet_point.d_state(0)<< " " <<
    //         kept_reference_point->frenet_point.s_state(0) << std::endl;
    double ref_last_waypoint_cost = 
    std::pow(frenet_point_at_time_horizon.d_state(0) - kept_reference_point->frenet_point.d_state(0), 2) + 
    std::pow(frenet_point_at_time_horizon.s_state(0) - kept_reference_point->frenet_point.s_state(0), 2);
    // std::pow(frenet_point_at_time_horizon.s_state(1) - kept_reference_point->frenet_point.s_state(1), 2);
    double sum_ref_waypoint_cost = 0;
    for(const auto& waypoint: trajectory.trajectory_points.waypoints)
    {
      autoware_msgs::Waypoint nearest_waypoint;
      getNearestWaypoint(waypoint.pose.pose.position,
                        reference_waypoints,
                        nearest_waypoint);
      double distance = calculate2DDistace(waypoint.pose.pose.position,
                        nearest_waypoint.pose.pose.position);
      sum_ref_waypoint_cost+= distance;
    }
    // std::cerr << "cost: sum\enet_point_at_time_horizon.d_state(0) << std::endl;
    sum_ref_waypoints_costs+= sum_ref_waypoint_cost;
    ref_waypoints_costs.push_back(sum_ref_waypoint_cost);
    
    // double ref_last_waypoint_cost = 
    // std::pow(frenet_point_at_time_horizon.d_state(0) - kept_reference_point->frenet_point.d_state(0), 2); 
    ref_last_waypoints_costs.push_back(ref_last_waypoint_cost);
    sum_last_waypoints_costs+= ref_last_waypoint_cost;
    debug_last_waypoints_costs_d.push_back(
      frenet_point_at_time_horizon.d_state(0) - kept_reference_point->frenet_point.d_state(0));
    debug_last_waypoints_costs_s.push_back(
      frenet_point_at_time_horizon.s_state(0) - kept_reference_point->frenet_point.s_state(0));
    debug_last_waypoints_actual_d.push_back(frenet_point_at_time_horizon.d_state(0));
    debug_last_waypoints_actual_s.push_back(frenet_point_at_time_horizon.s_state(0));
    // std::cerr << "diff s " <<  frenet_point_at_time_horizon.s_state(0) - kept_reference_point->frenet_point.s_state(0)<< std::endl;
    // std::cerr << "diff sv " <<  frenet_point_at_time_horizon.s_state(1) - kept_reference_point->frenet_point.s_state(1)<< std::endl;
    // std::cerr << "diff d " <<  frenet_point_at_time_horizon.d_state(0) - kept_reference_point->frenet_point.d_state(0)<< std::endl;
    // std::cerr << "total last wp diff " <<  ref_last_waypoint_cost<< std::endl;
    // std::cerr << "total wps diff " <<  ref_waypoints_cost<< std::endl;
  }
  
  std::vector<double> costs;
  std::vector<double> debug_norm_ref_wps_costs;
  std::vector<double> debug_norm_ref_last_wp_costs;
  std::vector<double> debug_norm_jerk_costs;
  std::vector<double> debug_norm_required_time_costs;
  // std::cerr << "sum las " << sum_last_waypoints_costs
            // << " sum jer "<< sum_jerk_costs
            // << " sum tim "<< sum_required_time_costs
            // <<std::endl;
  for(size_t i = 0; i < ref_last_waypoints_costs.size(); i++)
  {
    double normalized_ref_last_waypoints_cost = ref_last_waypoints_costs[i]/sum_last_waypoints_costs;
    double normalized_ref_waypoint_cost = ref_waypoints_costs[i]/sum_ref_waypoints_costs;
    
    
    double sum_cost = normalized_ref_last_waypoints_cost*diff_last_waypoint_cost_coef_ +
                      normalized_ref_waypoint_cost*diff_waypoints_cost_coef_;
    costs.push_back(sum_cost);
    // std::cerr << "sum cost " << i << " "<< sum_cost << std::endl;
    debug_norm_ref_last_wp_costs.push_back(normalized_ref_last_waypoints_cost);
    
    
  }
  //arg sort 
  // https://stackoverflow.com/questions/1577475/c-sorting-and-keeping-track-of-indexes/12399290#12399290
  std::vector<size_t> indexes(ref_last_waypoints_costs.size());
  std::iota(indexes.begin(), indexes.end(), 0);
  std::sort(indexes.begin(), indexes.end(), [&costs](const size_t &a, const size_t &b)
                                               { return costs[a] < costs[b];});
  
  bool has_got_best_trajectory = false;
  for(const auto& index: indexes)
  {
    // std::cerr << "------"  << std::endl;
    // std::cerr << "act las " << ref_last_waypoints_costs[index]
              // <<std::endl;
    bool is_collision_free = true;
    if(objects_ptr)
    {
      is_collision_free = isTrajectoryCollisionFree(
                            trajectories[index].trajectory_points.waypoints,
                            *objects_ptr);
    }
    
    if(is_collision_free)
    {
      has_got_best_trajectory = true;
      kept_best_trajectory.reset(new Trajectory(trajectories[index]));
      std::cerr << "ith traj " << index << std::endl;
      break;
    }
  } 
  
  //TODO: this might be bad effect
  if(!has_got_best_trajectory)
  {
    std::cerr << "ERROR: there is no trajectory"  << std::endl;
    return false;
  }
  else
  {
    return true;
  }
}

bool FrenetPlanner::isCollision(const autoware_msgs::Waypoint& waypoint,
                                const autoware_msgs::DetectedObjectArray& objects)
{
  //TODO: more sophisticated collision check
  for(const auto& object: objects.objects)
  {
    double distance = calculate2DDistace(waypoint.pose.pose.position,
                                          object.pose.position);
    //TODO: paremater
    //assuming obstacle is not car but corn
    if(distance < obstacle_radius_from_center_point_)
    {
      return true;
    }
  }
  return false;
}

bool FrenetPlanner::isCollision(const autoware_msgs::Waypoint& waypoint,
                                const autoware_msgs::DetectedObjectArray& objects,
                                size_t& collision_object_id,
                                size_t& collision_object_index)
{
  //TODO: more sophisticated collision check
  for(size_t i = 0; i < objects.objects.size(); i++)
  {
    double distance = calculate2DDistace(waypoint.pose.pose.position,
                                          objects.objects[i].pose.position);
    //TODO: paremater
    //assuming obstacle is not car but corn
    if(distance < 2.5)
    {
      collision_object_id = objects.objects[i].id;
      collision_object_index = i;
      return true;
    }
  }
  return false;
}

//TODO: not good interface; has 2 meanings check if safe, get collision waypoint
bool FrenetPlanner::isTrajectoryCollisionFree(
    const std::vector<autoware_msgs::Waypoint>& trajectory_points,
    const autoware_msgs::DetectedObjectArray& objects,
    size_t& collision_waypoint_index,
    size_t& collision_object_id,
    size_t& collision_object_index)
{
  for(size_t i= 0; i< trajectory_points.size(); i++)
  {
    size_t tmp_collision_object_id;
    size_t tmp_collision_object_index;
    bool is_collision = isCollision(trajectory_points[i], 
                                    objects,
                                    tmp_collision_object_id,
                                    tmp_collision_object_index);
    if(is_collision)
    {
      collision_waypoint_index = i;
      collision_object_id = tmp_collision_object_id;
      collision_object_index = tmp_collision_object_index;
      return false;
    }
  }
  return true;
}

//not sure this overload is good or bad
bool FrenetPlanner::isTrajectoryCollisionFree(
    const std::vector<autoware_msgs::Waypoint>& trajectory_points,
    const autoware_msgs::DetectedObjectArray& objects)
{
  for(const auto& point: trajectory_points)
  {
    bool is_collision = isCollision(point, objects);
    if(is_collision)
    {
      return false;
    }
  }
  return true;
}