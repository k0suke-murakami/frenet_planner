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

#ifndef MODIFIED_REFERENCE_PATH_GENERATOR_H
#define MODIFIED_REFERENCE_PATH_GENERATOR_H

namespace autoware_msgs
{
  ROS_DECLARE_MESSAGE(Waypoint); 
}

namespace grid_map
{
  class GridMap;
}

namespace geometry_msgs
{ 
  ROS_DECLARE_MESSAGE(TransformStamped);
}

struct PathPoint;

class ModifiedReferencePathGenerator
{
private:
  const double min_radius_;
  bool calculateCurvatureForPathPoints(
            std::vector<PathPoint>& path_points);
  double calculateCurvatureFromThreePoints(
          const Eigen::Vector2d& path_point1,
          const Eigen::Vector2d& path_point2,
          const Eigen::Vector2d& path_point3);
          
  double calculateSmoothness(
          const std::vector<PathPoint>& path_points);
          
  Eigen::Vector2d generateNewPosition(
          const Eigen::Vector2d& parent_of_path_point1,          
          const Eigen::Vector2d& path_point1,
          const Eigen::Vector2d& path_point2,
          const Eigen::Vector2d& path_point3,
          const grid_map::GridMap& clearance_map,
          const double min_r,
          const double max_k,
          const double resolution_of_gridmap);
          
  std::vector<double> generateOpenUniformKnotVector(
     const int number_of_knot,
     const int degree_of_b_spline);
     
  double calaculateBasisFunction(
     const std::vector<double>& knot_vector,
     const size_t control_point_index,
     const size_t basis_function_index,
     const double function_value);
          
public:
  ModifiedReferencePathGenerator(
     const double min_radius);
  ~ModifiedReferencePathGenerator();
  
  bool generateModifiedReferencePath(
      grid_map::GridMap& clearance_map,
      const geometry_msgs::Point& start_point,
      const geometry_msgs::Point& goal_point,
      const geometry_msgs::TransformStamped& lidar2map_tf,
      const geometry_msgs::TransformStamped& map2lidar_tf,
      std::vector<autoware_msgs::Waypoint>& modified_reference_path,
      std::vector<autoware_msgs::Waypoint>& debug_modified_smoothed_reference_path,      
      std::vector<autoware_msgs::Waypoint>& debug_bspline_path,      
      sensor_msgs::PointCloud2& debug_pointcloud_clearance_map);
};


#endif
