#ifndef IMLS_ICP_H
#define IMLS_ICP_H

#include <iostream>
#include <vector>
#include <set>
#include <nabo/nabo.h>


#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>
#include <eigen3/Eigen/Eigenvalues>
#include <eigen3/unsupported/Eigen/Polynomials>

#include <pcl-1.7/pcl/visualization/cloud_viewer.h>
#include <pcl-1.7/pcl/point_cloud.h>
#include <pcl-1.7/pcl/point_types.h>

#include <sys/time.h>


class IMLSICPMatcher
{
public:
    IMLSICPMatcher();
    IMLSICPMatcher(double _r,double _h,int _iter);
    ~IMLSICPMatcher();

    void setIterations(int _iter);

    void setSourcePointCloud(std::vector<Eigen::Vector2d>& _source_pcloud);

    void setSourcePointCloud(pcl::PointCloud<pcl::PointXYZ>& pcl_cloud);

    void setSourcePointCloudNormals(std::vector<Eigen::Vector2d>& _normals);

    void setTargetPointCloudNormals(std::vector<Eigen::Vector2d>& _normals);

    void setTargetPointCloud(std::vector<Eigen::Vector2d>& _target_pcloud);

    void setTargetPointCloud(pcl::PointCloud<pcl::PointXYZ>& pcl_cloud);

    void createKDTreeUsingLocalMap(void );


    // compute the height of projection
    bool ImplicitMLSFunction(Eigen::Vector2d x,
                             double& height);

    //把点云进行投影，投影到表面surface之中．
    // project the points to the target point cloud surface
    void projSourcePtToSurface(std::vector<Eigen::Vector2d>& in_cloud,
                                std::vector<Eigen::Vector2d>& out_cloud,
                                std::vector<Eigen::Vector2d> &out_normal);

    // solve 4 order polynomial
    bool SolverFourthOrderPolynomial(Eigen::VectorXd& p_coffi,
                                     double& lambda);

    // compute motion when matched points are known
    bool SolveMotionEstimationProblem(std::vector<Eigen::Vector2d>& source_cloud,
                                      std::vector<Eigen::Vector2d>& ref_cloud,
                                      std::vector<Eigen::Vector2d>& ref_normals,
                                      Eigen::Matrix3d& deltaTrans);

    Eigen::Vector2d ComputeNormal(std::vector<Eigen::Vector2d>& nearPoints);

    bool Match(Eigen::Matrix3d& finalPose,
               Eigen::Matrix3d& covariance);

private:

    void RemoveNANandINFData(std::vector<Eigen::Vector2d>& _input);

    // source point cloud and target point cloud
    // target point cloud is reference point cloud
    std::vector<Eigen::Vector2d> m_sourcePointCloud,m_targetPointCloud;

    // normal vectors
    std::vector<Eigen::Vector2d> m_sourcePtCloudNormals,m_targetPtCloudNormals;

    // each laser data and its id
    std::map<int,std::vector<int> > m_LaserFrames;

    // kd tree pointer
    Nabo::NNSearchD* m_pTargetKDTree;
    Nabo::NNSearchD* m_pSourceKDTree;

    Eigen::MatrixXd m_sourceKDTreeDataBase;

    Eigen::MatrixXd m_targetKDTreeDataBase;

    // iteration times．
    int m_Iterations;

    //id for point cloud．
    int m_PtID;

    int m_offsetBetweenLocalMapAndTree;

    //含义见论文．用来计算权重．
    //m_r ~ 3*m_h
    double m_h,m_r;

    bool m_isGetNormals;

};



#endif // IMLS_ICP_H
