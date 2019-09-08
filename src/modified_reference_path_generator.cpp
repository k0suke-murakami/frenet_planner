#include <autoware_msgs/Waypoint.h>
#include <grid_map_ros/grid_map_ros.hpp>
#include <geometry_msgs/TransformStamped.h>



#include "modified_reference_path_generator.h"

ModifiedReferencePathGenerator::ModifiedReferencePathGenerator(/* args */)
{
}

ModifiedReferencePathGenerator::~ModifiedReferencePathGenerator()
{
}

void ModifiedReferencePathGenerator::generateModifiedReferencePath(
  const grid_map::GridMap& clearance_map,
  const geometry_msgs::Point& start_point,
  const geometry_msgs::Point& goal_point,
  const geometry_msgs::TransformStamped& lidar2map_tf,
  std::vector<autoware_msgs::Waypoint>& modified_reference_path,
  sensor_msgs::PointCloud2& debug_pointcloud_clearance_map)
{
  
}
