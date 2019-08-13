#ifndef CALCULATE_CENTER_LINE_H
#define CALCULATE_CENTER_LINE_H


#include <autoware_msgs/Waypoint.h>
#include "vectormap_struct.h"


class CalculateCenterLine
{
private:
  /* data */
public:
  CalculateCenterLine(/* args */);
  ~CalculateCenterLine();
  
  std::vector<Point> calculateCenterLineFromGlobalWaypoints(
     const std::vector<autoware_msgs::Waypoint>& global_waypoints
  );
  
  // bool calculateCurvature(
  //   const std::vector<geometry_msgs::Point>& sampled_points
  // );
};


#endif