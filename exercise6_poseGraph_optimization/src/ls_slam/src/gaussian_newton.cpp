#include "gaussian_newton.h"
#include <eigen3/Eigen/Jacobi>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Householder>
#include <eigen3/Eigen/Cholesky>
#include <eigen3/Eigen/LU>

#include <iostream>


//位姿-->转换矩阵
Eigen::Matrix3d PoseToTrans(Eigen::Vector3d x)
{
    Eigen::Matrix3d trans;
    trans << cos(x(2)),-sin(x(2)),x(0),
             sin(x(2)), cos(x(2)),x(1),
                     0,         0,    1;

    return trans;
}


//转换矩阵－－＞位姿
Eigen::Vector3d TransToPose(Eigen::Matrix3d trans)
{
    Eigen::Vector3d pose;
    pose(0) = trans(0,2);
    pose(1) = trans(1,2);
    pose(2) = atan2(trans(1,0),trans(0,0));

    return pose;
}

//计算整个pose-graph的误差
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
 *         计算jacobian矩阵和error
 * @param xi    fromIdx
 * @param xj    toIdx
 * @param z     观测值:xj相对于xi的坐标
 * @param ei    计算的误差
 * @param Ai    相对于xi的Jacobian矩阵
 * @param Bi    相对于xj的Jacobian矩阵
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
 *        高斯牛顿方法的一次迭代．
 * @param Vertexs   图中的所有节点
 * @param Edges     图中的所有边
 * @return          位姿的增量
 */
Eigen::VectorXd  LinearizeAndSolve(std::vector<Eigen::Vector3d>& Vertexs,
                                   std::vector<Edge>& Edges)
{
    //申请内存
    Eigen::MatrixXd H(Vertexs.size() * 3,Vertexs.size() * 3);
    Eigen::VectorXd b(Vertexs.size() * 3);

    H.setZero();
    b.setZero();

    //固定第一帧
    Eigen::Matrix3d I;
    I.setIdentity();
    H.block(0,0,3,3) += I;

    //构造H矩阵　＆ b向量
    for(int i = 0; i < Edges.size();i++)
    {
        //提取信息
        Edge tmpEdge = Edges[i];
        Eigen::Vector3d xi = Vertexs[tmpEdge.xi];
        Eigen::Vector3d xj = Vertexs[tmpEdge.xj];
        Eigen::Vector3d z = tmpEdge.measurement;
        Eigen::Matrix3d infoMatrix = tmpEdge.infoMatrix;

        //计算误差和对应的Jacobian
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

    //求解
    Eigen::VectorXd dx;

    dx = -H.lu().solve(b);

    return dx;
}











