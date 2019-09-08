#include <autoware_msgs/Waypoint.h>

#include <geometry_msgs/TransformStamped.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <grid_map_ros/GridMapRosConverter.hpp>
#include <grid_map_ros/grid_map_ros.hpp>
#include <grid_map_msgs/GridMap.h>

#include <distance_transform/distance_transform.hpp>





#include "modified_reference_path_generator.h"

ModifiedReferencePathGenerator::ModifiedReferencePathGenerator(/* args */)
{
}

ModifiedReferencePathGenerator::~ModifiedReferencePathGenerator()
{
}

void ModifiedReferencePathGenerator::generateModifiedReferencePath(
  grid_map::GridMap& clearance_map,
  const geometry_msgs::Point& start_point,
  const geometry_msgs::Point& goal_point,
  const geometry_msgs::TransformStamped& lidar2map_tf,
  const geometry_msgs::TransformStamped& map2lidar_tf,
  std::vector<autoware_msgs::Waypoint>& modified_reference_path,
  sensor_msgs::PointCloud2& debug_pointcloud_clearance_map)
{
  std::string layer_name = clearance_map.getLayers().back();
  grid_map::Matrix data = clearance_map.get(layer_name);
  
  //grid_length y and grid_length_x respectively
  dope::Index2 size({100, 300});
  dope::Grid<float, 2> f(size);
  dope::Grid<dope::SizeType, 2> indices(size);
  bool is_empty_cost = true;
  for (dope::SizeType i = 0; i < size[0]; ++i)
  {
    for (dope::SizeType j = 0; j < size[1]; ++j) 
    {
        if (data(i*size[1] + j) > 0.01)
        {
          f[i][j] = 0.0f;
          is_empty_cost = false;
        }
        else
        {
          f[i][j] = std::numeric_limits<float>::max();
        }
    }
  }

  // Note: this is necessary at least at the first distance transform execution
  // and every time a reset is desired; it is not, instead, when updating
  dt::DistanceTransform::initializeIndices(indices);
  dt::DistanceTransform::distanceTransformL2(f, f, false, 1);
  
  
  for (dope::SizeType i = 0; i < size[0]; ++i)
  {
    for (dope::SizeType j = 0; j < size[1]; ++j) 
    {
      if(is_empty_cost)
      {
        data(i*size[1] + j) = 1;
      }
      else
      {
        data(i*size[1] + j) = f[i][j];
      }
    }
  }
  
  clearance_map[layer_name] = data;
  grid_map::GridMapRosConverter::toPointCloud(clearance_map,
                                              layer_name,
                                              debug_pointcloud_clearance_map);
  
}
