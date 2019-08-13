#ifndef VECTORMAP_ROS_H
#define VECTORMAP_ROS_H

#include <vector>
// #include <stack>
#include <queue>
#include <unordered_map>

#include "vector_map_msgs/PointArray.h"
#include "vector_map_msgs/DTLaneArray.h"
#include "vector_map_msgs/NodeArray.h"
#include "vector_map_msgs/LaneArray.h"

#include "vectormap_struct.h"

//TODO: less depedency on header



class VectorMap
{
  public:

    bool load();
    std::vector<Point> points_;  
};

#endif