#include <Eigen/Dense>

#include "calculate_center_line.h"

CalculateCenterLine::CalculateCenterLine(/* args */)
{
}

CalculateCenterLine::~CalculateCenterLine()
{
}

std::vector<Point> CalculateCenterLine::calculateCenterLineFromGlobalWaypoints(
     const std::vector<autoware_msgs::Waypoint>& global_waypoints)
{
  const int num_sampling_points = 3;
  const int index_offset = (num_sampling_points-1)/2;
  std::vector<Point> center_line_points;
  for(size_t i = 0; i < global_waypoints.size(); i++)
  {
    Point center_line_point;
    center_line_point.tx = global_waypoints[i].pose.pose.position.x;
    center_line_point.ty = global_waypoints[i].pose.pose.position.y;
    
    
    //calculate curvature
    const int first_index = i - index_offset;
    const int last_index = i + index_offset;
    double curvature = 0;
    if(first_index <= 0 || last_index >= global_waypoints.size())
    {
      curvature = 0.0;
    }
    else
    {
      std::vector<geometry_msgs::Point> sampled_points;
      int lookup_index = first_index;
      for(size_t j= 0; j< num_sampling_points; j++)
      {
        geometry_msgs::Point sampled_point 
        = global_waypoints[lookup_index].pose.pose.position;
        sampled_points.push_back(sampled_point);
        lookup_index++;
      }
      
      // use Cramer's rule for solving linear equation
      // https://www.xarg.org/2018/02/create-a-circle-out-of-three-points/
      // only applicable to 3 sampling points

      double x1 = sampled_points.at(0).x;
      double y1 = sampled_points.at(0).y;
      double x2 = sampled_points.at(1).x;
      double y2 = sampled_points.at(1).y;
      double x3 = sampled_points.at(2).x;
      double y3 = sampled_points.at(2).y;
      
      double a =  x1 * (y2 - y3) - y1 * (x2 - x3) + x2 * y3 - x3 * y2;

      double b = (x1 * x1 + y1 * y1) * (y3 - y2) 
              + (x2 * x2 + y2 * y2) * (y1 - y3)
              + (x3 * x3 + y3 * y3) * (y2 - y1);
  
      double c = (x1 * x1 + y1 * y1) * (x2 - x3) 
              + (x2 * x2 + y2 * y2) * (x3 - x1) 
              + (x3 * x3 + y3 * y3) * (x1 - x2);
  
      
      double x = -b / (2 * a);
      double y = -c / (2 * a);
      double r = std::sqrt(std::pow(x - x1, 2) + std::pow(y - y1, 2));
      
      // using cross product
      // https://stackoverflow.com/questions/243945/calculating-a-2d-vectors-cross-product
      double plus_or_minus_sign = (x1 - x2)*(y3 - y2) - (y1 - y2)*(x3 - x2);
      Eigen::Vector2d v1, v2;
      v1 << x1 - x2, y1 - y2;
      v2 << x3 - x2, y3 - y2;
      double yaw_by_cos = std::acos(v1.dot(v2)/(v1.norm()*v2.norm()));
      
      double tmp_curvature = 0;
      //straight check
      if(yaw_by_cos > (M_PI- 0.01))
      {
        tmp_curvature = 0;
      }
      else if(plus_or_minus_sign > 0)
      {
        tmp_curvature = -1/r;
      }
      else
      {
        tmp_curvature = 1/r;
      }
      curvature = tmp_curvature;
    }
    // std::cerr << "curvarture " << curvature << std::endl;
    center_line_point.curvature = curvature;

    //calculate cumulated_s
    //calculate curvature_dot
    double yaw = 0;
    double cumulted_s = 0;
    double curvature_dot = 0;
    if(center_line_points.size()==0)
    {
      cumulted_s = 0;
      curvature_dot = 0;
      yaw = 0;
    }
    else
    {
      Point previous_point = center_line_points.back();
      double p_x = previous_point.tx;
      double p_y = previous_point.ty;
      double c_x = global_waypoints[i].pose.pose.position.x;
      double c_y = global_waypoints[i].pose.pose.position.y;
      yaw = std::atan2(c_y - p_y, c_x - p_x);
      double delta_distance = std::sqrt(std::pow(p_x - c_x, 2) + std::pow(p_y - c_y, 2));
      cumulted_s = delta_distance + previous_point.cumulated_s;
      curvature_dot = center_line_point.curvature/delta_distance;
    }
    center_line_point.rz = yaw;
    center_line_point.cumulated_s = cumulted_s;
    center_line_point.curvature_dot = curvature_dot;
    
    center_line_points.push_back(center_line_point);
  }
  return center_line_points;
}

