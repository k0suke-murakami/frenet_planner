
#ifndef VECTORMAP_STRUCT_H
#define VECTORMAP_STRUCT_H

#include <vector>



struct Point
{
    double tx;  // translation of X axis
    double ty;  // translation of Y axis
    double rz;  // rotation of Z axis (yaw)
    // double sr;  // squared radius
    double curvature;
    double curvature_dot;
    double cumulated_s; // cumulated s in frenet coordinate from the origin lane point 
    std::vector<Point> points;
};


#endif