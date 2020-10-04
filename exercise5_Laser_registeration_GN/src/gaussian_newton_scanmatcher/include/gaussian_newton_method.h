#ifndef GAUSSIAN_NEWTON_METHOD_H
#define GAUSSIAN_NEWTON_METHOD_H

#include <cstdio>
#include <vector>
#include <cmath>
#include <iostream>

#include "map.h"

#include "eigen3/Eigen/Core"
#include "eigen3/Eigen/Dense"

// creat likelihood field using laser data
map_t* CreateMapFromLaserPoints(Eigen::Vector3d map_origin_pt,
                              std::vector<Eigen::Vector2d> laser_pts,
                              double resolution);

// use GN to optimize pose
void GaussianNewtonOptimization(map_t*map,Eigen::Vector3d& init_pose,std::vector<Eigen::Vector2d>& laser_pts);


#endif // GAUSSIAN_NEWTON_METHOD_H
