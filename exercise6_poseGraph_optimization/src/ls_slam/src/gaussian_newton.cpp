#include "gaussian_newton.h"
#include <eigen3/Eigen/Jacobi>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Householder>
#include <eigen3/Eigen/Cholesky>
#include <eigen3/Eigen/LU>

#include <iostream>


// pose -> transform matrix
Eigen::Matrix3d PoseToTrans(Eigen::Vector3d x)
{
    Eigen::Matrix3d trans;
    trans << cos(x(2)),-sin(x(2)),x(0),
             sin(x(2)), cos(x(2)),x(1),
                     0,         0,    1;

    return trans;
}


// transform matrix -> pose
Eigen::Vector3d TransToPose(Eigen::Matrix3d trans)
{
    Eigen::Vector3d pose;
    pose(0) = trans(0,2);
    pose(1) = trans(1,2);
    pose(2) = atan2(trans(1,0),trans(0,0));

    return pose;
}

// compute error of pose graph
double ComputeError(std::vector<Eigen::Vector3d>& Vertexs,
                    std::vector<Edge>& Edges)
{
    double sumError = 0;
    for(int i = 0; i < Edges.size();i++)
    {
        Edge tmpEdge = Edges[i];
        Eigen::Vector3d xi = Vertexs[tmpEdge.xi];
        Eigen::Vector3d xj = Vertexs[tmpEdge.xj];
        Eigen::Vector3d z = tmpEdge.measurement;
        Eigen::Matrix3d infoMatrix = tmpEdge.infoMatrix;

        Eigen::Matrix3d Xi = PoseToTrans(xi);
        Eigen::Matrix3d Xj = PoseToTrans(xj);
        Eigen::Matrix3d Z  = PoseToTrans(z);

        Eigen::Matrix3d Ei = Z.inverse() *  Xi.inverse() * Xj;

        Eigen::Vector3d ei = TransToPose(Ei);


        sumError += ei.transpose() * infoMatrix * ei;
    }
    return sumError;
}


/**
 * @brief CalcJacobianAndError
 *        compute jacobian matrix and error
 * @param xi    fromIdx
 * @param xj    toIdx
 * @param z     the coordinate of xj w.r.t frame xi
 * @param ei    error
 * @param Ai    Jacobian matrix w.r.t xi
 * @param Bi    Jacobian matrix w.r.t xj
 */
void CalcJacobianAndError(Eigen::Vector3d xi,Eigen::Vector3d xj,Eigen::Vector3d z,
                          Eigen::Vector3d& ei,Eigen::Matrix3d& Ai,Eigen::Matrix3d& Bi)
{
    Eigen::Matrix3d Xi = PoseToTrans(xi);
    Eigen::Matrix3d Xj = PoseToTrans(xj);
    Eigen::Matrix3d Z = PoseToTrans(z);

    Eigen::Matrix3d E = Z.inverse() * Xi.inverse() * Xj;
    ei = TransToPose(E);
    
    // compute Jacobian
    Eigen::Matrix2d Rij = Z.block(0,0,2,2);
    Eigen::Vector2d tij = Z.block(0,2,2,1);
    Eigen::Matrix2d Ri = Xi.block(0,0,2,2);
    Eigen::Vector2d ti = Xi.block(0,2,2,1);
    Eigen::Vector2d tj = Xj.block(0,2,2,1);
    Eigen::Matrix2d dRi;
    double theta = xi(2);
    dRi << -sin(theta), cos(theta),
            -cos(theta), -sin(theta);
    Ai.setZero();
    Ai.block(0,0,2,2) = -Rij.transpose()*Ri.transpose();
    Ai.block(0,2,2,1) = Rij.transpose()*dRi*(tj-ti);
    Ai(2,2) = -1; 

    Bi.setZero();
    Bi.block(0,0,2,2) = Rij.transpose()*Ri.transpose();
    Bi(2,2) = 1;

}

/**
 * @brief LinearizeAndSolve
 *        one iteration in Gauss-Newton
 * @param Vertexs   Verticies in graph
 * @param Edges     edges in graph
 * @return          increment of pose
 */
Eigen::VectorXd  LinearizeAndSolve(std::vector<Eigen::Vector3d>& Vertexs,
                                   std::vector<Edge>& Edges)
{

    Eigen::MatrixXd H(Vertexs.size() * 3,Vertexs.size() * 3);
    Eigen::VectorXd b(Vertexs.size() * 3);

    H.setZero();
    b.setZero();

    // fix the first frame
    Eigen::Matrix3d I;
    I.setIdentity();
    H.block(0,0,3,3) += I;

    // construct H and b
    for(int i = 0; i < Edges.size();i++)
    {
        Edge tmpEdge = Edges[i];
        Eigen::Vector3d xi = Vertexs[tmpEdge.xi];
        Eigen::Vector3d xj = Vertexs[tmpEdge.xj];
        Eigen::Vector3d z = tmpEdge.measurement;
        Eigen::Matrix3d infoMatrix = tmpEdge.infoMatrix;

        // compute error and jacobian
        Eigen::Vector3d ei;
        Eigen::Matrix3d Ai;
        Eigen::Matrix3d Bi;
        CalcJacobianAndError(xi,xj,z,ei,Ai,Bi);

        Eigen::Matrix3d Hii, Hij, Hji, Hjj;
        Hii = Ai.transpose()*infoMatrix*Ai;
        Hij = Ai.transpose()*infoMatrix*Bi;
        Hji = Bi.transpose()*infoMatrix*Ai;
        Hjj = Bi.transpose()*infoMatrix*Bi;

        int idx_i = tmpEdge.xi;
        int idx_j = tmpEdge.xj;

        H.block(3*idx_i,3*idx_i, 3, 3) += Hii;
        H.block(3*idx_i,3*idx_j, 3, 3) += Hij;
        H.block(3*idx_j,3*idx_i, 3, 3) += Hji;
        H.block(3*idx_j,3*idx_j, 3, 3) += Hjj;
        

        Eigen::Vector3d bi, bj;
        bi = Ai.transpose() * infoMatrix * ei;
        bj = Bi.transpose() * infoMatrix * ei;

        b.segment(idx_i*3, 3) += bi;
        b.segment(idx_j*3, 3) += bj;

    }

    Eigen::VectorXd dx;
    dx = -H.lu().solve(b);

    return dx;
}



