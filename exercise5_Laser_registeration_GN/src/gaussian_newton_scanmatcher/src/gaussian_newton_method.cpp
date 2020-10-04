#include <map.h>
#include "gaussian_newton_method.h"

const double GN_PI = 3.1415926;

double GN_NormalizationAngle(double angle)
{
    if(angle > GN_PI)
        angle -= 2*GN_PI;
    else if(angle < -GN_PI)
        angle += 2*GN_PI;

    return angle;
}

Eigen::Matrix3d GN_V2T(Eigen::Vector3d vec)
{
    Eigen::Matrix3d T;
    T  << cos(vec(2)),-sin(vec(2)),vec(0),
            sin(vec(2)), cos(vec(2)),vec(1),
            0,           0,     1;

    return T;
}

Eigen::Vector2d GN_TransPoint(Eigen::Vector2d pt,Eigen::Matrix3d T)
{
    Eigen::Vector3d tmp_pt(pt(0),pt(1),1);
    tmp_pt = T * tmp_pt;
    return Eigen::Vector2d(tmp_pt(0),tmp_pt(1));
}



// use laser data to generate a likelihood field
map_t* CreateMapFromLaserPoints(Eigen::Vector3d map_origin_pt,
                                std::vector<Eigen::Vector2d> laser_pts,
                                double resolution)
{
    map_t* map = map_alloc();

    map->origin_x = map_origin_pt(0);
    map->origin_y = map_origin_pt(1);
    map->resolution = resolution;

    // map size
    map->size_x = 10000;
    map->size_y = 10000;

    map->cells = (map_cell_t*)malloc(sizeof(map_cell_t)*map->size_x*map->size_y);

    map->likelihood_sigma = 0.5;

    Eigen::Matrix3d Trans = GN_V2T(map_origin_pt);

    // set obstacle
    for(int i = 0; i < laser_pts.size();i++)
    {
        Eigen::Vector2d tmp_pt = GN_TransPoint(laser_pts[i],Trans);

        int cell_x,cell_y;
        cell_x = MAP_GXWX(map,tmp_pt(0));
        cell_y = MAP_GYWY(map,tmp_pt(1));

        map->cells[MAP_INDEX(map,cell_x,cell_y)].occ_state = CELL_STATUS_OCC;
    }

    // dilation of obstacles, max distance is 0.5
    map_update_cspace(map,0.5);

    return map;
}


/**
 * @brief InterpMapValueWithDerivatives
 * map interpolation, get the gradient at coords
 * ans(0) is likelihood value
 * ans(1:2) is gradient w.r.t position
 * @param map
 * @param coords
 * @return
 */
Eigen::Vector3d InterpMapValueWithDerivatives(map_t* map,Eigen::Vector2d& coords)
{
    Eigen::Vector3d ans;
    // Get the left top of current point (coordinate of Z1)
    Eigen::Vector2i xy_norm = Eigen::Vector2i::Zero();
    xy_norm << floor((coords.x() - map->origin_x) / map->resolution) + map->size_x/2,
                floor((coords.y() - map->origin_y) / map->resolution) + map->size_y/2;
    
    if(!MAP_VALID(map, xy_norm.x(), xy_norm.y())){
        return Eigen::Vector3d::Zero();
    }

    // Transform xy_norm to u,v Frame
    const double u = (coords.x() - map->origin_x) / map->resolution + map->size_x/2 - xy_norm.x();
    const double v = (coords.y() - map->origin_y) / map->resolution + map->size_y/2 - xy_norm.y();

    const double z1 = map->cells[MAP_INDEX(map, xy_norm.x(), xy_norm.y())].score;
    const double z2 = map->cells[MAP_INDEX(map, xy_norm.x()+1, xy_norm.y())].score;
    const double z3 = map->cells[MAP_INDEX(map, xy_norm.x()+1, xy_norm.y()+1)].score;
    const double z4 = map->cells[MAP_INDEX(map, xy_norm.x(), xy_norm.y()+1)].score;

    ans(0) = (1.0-u)*(1-v)*z1 + u*(1.0-v)*z2 + u*v*z3 + (1.0-u)*v*z4;
    ans(1) = (v*(z3-z4) + (1.0-v)*(z2-z1)) / map->resolution;
    ans(2) = (u*(z3-z2) + (1.0-u)*(z4-z1)) / map->resolution;

    return ans;
}


/**
 * @brief ComputeCompleteHessianAndb
 * compute H and b for Hx = b
 * @param map
 * @param now_pose
 * @param laser_pts
 * @param H
 * @param b
 */
double ComputeHessianAndb(map_t* map, Eigen::Vector3d now_pose,
                        std::vector<Eigen::Vector2d>& laser_pts,
                        Eigen::Matrix3d& H, Eigen::Vector3d& b)
{
    H = Eigen::Matrix3d::Zero();
    b = Eigen::Vector3d::Zero();
    double residual = 0;

    Eigen::MatrixXd d_Si2T = Eigen::MatrixXd::Zero(2,3);
    d_Si2T.block(0,0,2,2) << 1, 0, 0, 1;
    Eigen::Matrix3d pose_matrix = GN_V2T(now_pose);

    for(auto point : laser_pts){
        // Transform Laser Point to World Frame
        Eigen::Vector2d p_world = GN_TransPoint(point, pose_matrix);

        d_Si2T(0,2) = -sin(now_pose(2)) * point.x() - cos(now_pose(2)) * point.y();
        d_Si2T(1,2) = cos(now_pose(2)) * point.x() - sin(now_pose(2)) * point.y();

        // Get interpolate value and its derivative
        // m_Si(0) is the value in likelihood field
        // m_Si(1:2) are derivative with respect to the position
        Eigen::Vector3d m_Si = InterpMapValueWithDerivatives(map, p_world);

        // Get H and b
        Eigen::RowVector3d J = m_Si.tail(2).transpose() * d_Si2T;

        H += J.transpose() * J;
        b += (1.0 - m_Si(0)) * J.transpose();
        
        residual += pow(1.0 - m_Si(0), 2);

    }

    //END OF TODO
    return residual;
}


/**
 * @brief GaussianNewtonOptimization
 * Gauss-Newton optimization
 * @param map
 * @param init_pose
 * @param laser_pts
 */
void GaussianNewtonOptimization(map_t*map,Eigen::Vector3d& init_pose,std::vector<Eigen::Vector2d>& laser_pts)
{
    int maxIteration = 20;
    Eigen::Vector3d now_pose = init_pose;

    double scale = 0.6;
    double epsilon = 1e-6;
    double min_residual = std::numeric_limits<double>::max();

    for(int i = 0; i < maxIteration;i++)
    {
        Eigen::Matrix3d H = Eigen::Matrix3d::Zero();
        Eigen::Vector3d b = Eigen::Vector3d::Zero();

        double residual = ComputeHessianAndb(map, now_pose, laser_pts, H, b);
        if(min_residual < residual || fabs(min_residual - residual) < epsilon)
            break;
            
        min_residual = residual;

        Eigen::Vector3d delta = H.ldlt().solve(b) * scale;

        now_pose +=delta;
    }
    init_pose = now_pose;

}
