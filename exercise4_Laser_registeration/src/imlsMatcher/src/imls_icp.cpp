#include "imls_icp.h"


IMLSICPMatcher::IMLSICPMatcher(void )
{
    m_r = 0.03;
    m_h = 0.10;
    m_Iterations = 10;

    m_PtID = 0;

    m_pTargetKDTree = m_pSourceKDTree = NULL;
}

IMLSICPMatcher::IMLSICPMatcher(double _r,double _h,int _iter)
{
    m_r = _r;
    m_h = _h;
    m_Iterations = _iter;
}

IMLSICPMatcher::~IMLSICPMatcher(void )
{
    if(m_pTargetKDTree != NULL)
        delete m_pTargetKDTree;

    if(m_pSourceKDTree != NULL)
        delete m_pSourceKDTree;
}

void IMLSICPMatcher::setIterations(int _iter)
{
    m_Iterations = _iter;
}

// remove outliers
void IMLSICPMatcher::RemoveNANandINFData(std::vector<Eigen::Vector2d> &_input)
{

    for(std::vector<Eigen::Vector2d>::iterator it = _input.begin();it != _input.end();)
    {
        Eigen::Vector2d tmpPt = *it;
        if(std::isnan(tmpPt(0)) || std::isnan(tmpPt(1)) ||
                std::isinf(tmpPt(0)) || std::isinf(tmpPt(1)))
        {
            it = _input.erase(it);
        }
        else
        {
            it++;
        }
    }
}


// set current point cloud
void IMLSICPMatcher::setSourcePointCloud(std::vector<Eigen::Vector2d> &_source_pcloud)
{
    m_sourcePointCloud = _source_pcloud;

    // remove outliers
    RemoveNANandINFData(m_sourcePointCloud);
}


void IMLSICPMatcher::setSourcePointCloudNormals(std::vector<Eigen::Vector2d> &_normals)
{
    m_sourcePtCloudNormals = _normals;
}


// set previous point cloud
void IMLSICPMatcher::setTargetPointCloud(std::vector<Eigen::Vector2d> &_target_pcloud)
{
    m_targetPointCloud = _target_pcloud;

    if(m_pTargetKDTree != NULL)
    {
        delete m_pTargetKDTree;
        m_pTargetKDTree = NULL;
    }

    RemoveNANandINFData(m_targetPointCloud);


    // construct kd tree for knn search
    if(m_pTargetKDTree == NULL)
    {
        m_targetKDTreeDataBase.resize(2,m_targetPointCloud.size());
        for(int i = 0; i < m_targetPointCloud.size();i++)
        {
            m_targetKDTreeDataBase(0,i) = m_targetPointCloud[i](0);
            m_targetKDTreeDataBase(1,i) = m_targetPointCloud[i](1);
        }
        m_pTargetKDTree = Nabo::NNSearchD::createKDTreeLinearHeap(m_targetKDTreeDataBase);
    }

    // need to compute normal vectors
    m_isGetNormals = false;
}


// IMLS function
// compute the height of projection
bool IMLSICPMatcher::ImplicitMLSFunction(Eigen::Vector2d x,
                                         double& height)
{
    double weightSum = 0.0;
    double projSum = 0.0;


    if(m_pTargetKDTree == NULL)
    {
        m_targetKDTreeDataBase.resize(2,m_targetPointCloud.size());
        for(int i = 0; i < m_targetPointCloud.size();i++)
        {
            m_targetKDTreeDataBase(0,i) = m_targetPointCloud[i](0);
            m_targetKDTreeDataBase(1,i) = m_targetPointCloud[i](1);
        }
        m_pTargetKDTree = Nabo::NNSearchD::createKDTreeLinearHeap(m_targetKDTreeDataBase);
    }

    // find 20 points near to x
    int searchNumber = 20;
    Eigen::VectorXi nearIndices(searchNumber);
    Eigen::VectorXd nearDist2(searchNumber);

    // max searching distance is m_h
    m_pTargetKDTree->knn(x,nearIndices,nearDist2,searchNumber,0,
                         Nabo::NNSearchD::SORT_RESULTS | Nabo::NNSearchD::ALLOW_SELF_MATCH|
                         Nabo::NNSearchD::TOUCH_STATISTICS,
                         m_h);

    std::vector<Eigen::Vector2d> nearPoints;
    std::vector<Eigen::Vector2d> nearNormals;
    for(int i = 0; i < searchNumber;i++)
    {
        // valid nearest neighbor
        if(nearDist2(i) < std::numeric_limits<double>::infinity() &&
                std::isinf(nearDist2(i)) == false &&
                std::isnan(nearDist2(i)) == false)
        {
            int index = nearIndices(i);

            Eigen::Vector2d tmpPt(m_targetKDTreeDataBase(0,index),m_targetKDTreeDataBase(1,index));

            if(std::isinf(tmpPt(0))||std::isinf(tmpPt(1))||
                    std::isnan(tmpPt(0))||std::isnan(tmpPt(1)))
            {
                continue;
            }

            Eigen::Vector2d normal;
            normal = m_targetPtCloudNormals[index];

            if(std::isinf(normal(0))||std::isinf(normal(1))||
                    std::isnan(normal(0))||std::isnan(normal(1)))
            {
                continue;
            }

            nearPoints.push_back(tmpPt);
            nearNormals.push_back(normal);
        }
        else
        {
            break;
        }
    }

    // if nearest points too few, reject this
    if(nearPoints.size() < 3)
    {
        return false;
    }


    // compute height I(x)
    double weight_sum = 0;
    double proj_sum = 0;

    for(int i=0; i<nearPoints.size(); i++){
        Eigen::Vector2d pt = nearPoints[i];
        Eigen::Vector2d normal = nearNormals[i];

        double temp = (x-pt).norm();

        double weight = std::exp(-temp*temp/(m_h*m_h));
        proj_sum += weight * (x-pt).dot(normal);
        weight_sum += weight;
    }

    height = proj_sum / weight_sum;

    return true;
}



/**
 * @brief IMLSICPMatcher::projSourcePtToSurface
    this function projects the points from transformed source point cloud to target point cloud surface.
    for each point in transformed source point cloud, find a projected point and its normal vector
    at the same time, points from source point cloud but can't find projected points are eliminated


 * @param in_cloud          transformed source point cloud
 * @param out_cloud         projected point cloud
 * @param out_normal        normal vectors of projected point cloud．
 */
void IMLSICPMatcher::projSourcePtToSurface(
        std::vector<Eigen::Vector2d> &in_cloud,
        std::vector<Eigen::Vector2d> &out_cloud,
        std::vector<Eigen::Vector2d> &out_normal)
{
    out_cloud.clear();
    out_normal.clear();

    for(std::vector<Eigen::Vector2d>::iterator it = in_cloud.begin(); it != in_cloud.end();)
    {
        Eigen::Vector2d xi = *it;

        // find nearest neighbor in target point cloud
        int K = 1;
        Eigen::VectorXi indices(K);
        Eigen::VectorXd dist2(K);
        m_pTargetKDTree->knn(xi,indices,dist2);

        Eigen::Vector2d nearXi = m_targetKDTreeDataBase.col(indices(0));

        Eigen::Vector2d nearNormal = m_targetPtCloudNormals[indices(0)];

        // if matched point doesn't have normal vector, remove it
        if(std::isinf(nearNormal(0))||std::isinf(nearNormal(1))||
                std::isnan(nearNormal(0))||std::isnan(nearNormal(1)))
        {
            it = in_cloud.erase(it);
            continue;
        }

        // if matched point is too far, remove
        if(dist2(0) > m_h * m_h )
        {
            it = in_cloud.erase(it);
            continue;
        }

        // compute the height
        double height;
        if(ImplicitMLSFunction(xi,height) == false)
        {
            it = in_cloud.erase(it);
            continue;
        }

        if(std::isnan(height))
        {
            std::cout <<"proj:this is not possible"<<std::endl;
            it = in_cloud.erase(it);
            continue;
        }

        if(std::isinf(height))
        {
            std::cout <<"proj:this is inf,not possible"<<std::endl;
            it = in_cloud.erase(it);
            continue;
        }

        Eigen::Vector2d yi;
        yi = xi - height * nearNormal;

        out_cloud.push_back(yi);
        out_normal.push_back(nearNormal);

        it++;
    }
}


// known matched points and normal vectors, compute transformation matrix
// point-to-line icp
bool IMLSICPMatcher::SolveMotionEstimationProblem(std::vector<Eigen::Vector2d> &source_cloud,
                                                  std::vector<Eigen::Vector2d> &ref_cloud,
                                                  std::vector<Eigen::Vector2d> &ref_normals,
                                                  Eigen::Matrix3d& deltaTrans)
{
    Eigen::Matrix4d M;
    M.setZero();
    Eigen::Matrix<double,1,4> gt; //gt is a row vector
    gt.setZero();

    for(int i = 0; i < source_cloud.size();i++)
    {
        //source point
        Eigen::Vector2d p = source_cloud[i];

        //target-point
        Eigen::Vector2d refPt = ref_cloud[i];

        Eigen::Vector2d ni = ref_normals[i];

        // weight matrix
        // for point-to-point, Ci =wi * I
        // for point-to-line, Ci =wi *  n*n^T
        Eigen::Matrix2d Ci =ni * ni.transpose();

        // construct M matrix
        Eigen::Matrix<double,2,4> Mi;
        Mi <<   1,0,p(0),-p(1),
                0,1,p(1), p(0);
        M += Mi.transpose() * Ci * Mi;

        Eigen::Matrix<double,1,4> gti;
        gti  = -2 * refPt.transpose() * Ci * Mi;

        gt += gti;
    }

    //g is a column vector
    Eigen::Matrix<double,4,1> g = gt.transpose();

    //构建完了M矩阵和g向量．
    //在后续的求解过程中，基本上都使用的是2*M,因此直接令M = 2*M
    M = 2*M;

    //M(actually 2*M) can be decomposed to 4 parts
    Eigen::Matrix2d A,B,D;
    A = M.block(0,0,2,2);
    B = M.block(0,2,2,2);
    D = M.block(2,2,2,2);

    // paper also involves S and SA matrix
    Eigen::Matrix2d S,SA;
    S = D - B.transpose() * A.inverse() * B;
    SA = S.determinant() * S.inverse();

    //目前所有的式子已经可以构建多项式系数了．
    //式31右边p(\lambda)的系数
    //p(\lambda)的系数为：
    Eigen::Vector3d p_coffcient;
    p_coffcient << S.determinant(),2*(S(0,0)+S(1,1)) ,4;

    //论文中式(31)左边的系数(a x^2 + b x + c)为：
    double a,b,c;
    Eigen::Matrix4d tmpMatrix;
    tmpMatrix.block(0,0,2,2) = A.inverse() * B  * SA  * SA.transpose()* B.transpose() * A.inverse().transpose();
    tmpMatrix.block(0,2,2,2) = -A.inverse() * B  * SA * SA.transpose();
    tmpMatrix.block(2,0,2,2) = tmpMatrix.block(0,2,2,2).transpose();
    tmpMatrix.block(2,2,2,2) = SA * SA.transpose() ;

    c = g.transpose() * tmpMatrix * g;

    tmpMatrix.block(0,0,2,2) = A.inverse() * B * SA * B.transpose() * A.inverse().transpose();
    tmpMatrix.block(0,2,2,2) = -A.inverse() * B * SA;
    tmpMatrix.block(2,0,2,2) = tmpMatrix.block(0,2,2,2).transpose();
    tmpMatrix.block(2,2,2,2) = SA;
    b = 4 * g.transpose() * tmpMatrix * g;

    tmpMatrix.block(0,0,2,2) = A.inverse() * B * B.transpose() * A.inverse().transpose();
    tmpMatrix.block(0,2,2,2) = -A.inverse() * B;
    tmpMatrix.block(2,0,2,2) = tmpMatrix.block(0,2,2,2).transpose();
    tmpMatrix.block(2,2,2,2) = Eigen::Matrix2d::Identity();
    a = 4 * g.transpose() * tmpMatrix * g;

    //把式31的等式两边进行合并，得到一个4次多项式．５个系数．
    Eigen::VectorXd poly_coffi(5);
    poly_coffi(0) = c - p_coffcient(0) * p_coffcient(0);
    poly_coffi(1) = b - 2 * p_coffcient(0) * p_coffcient(1);
    poly_coffi(2) = a - (p_coffcient(1)*p_coffcient(1) + 2*p_coffcient(0)*p_coffcient(2));
    poly_coffi(3) = -2 * p_coffcient(1)*p_coffcient(2);
    poly_coffi(4) = - p_coffcient(2) * p_coffcient(2);

    for(int i = 0; i < 5;i++)
    {
        if(std::isnan(poly_coffi(i)))
        {
            std::cout <<"Error, This should not happen"<<std::endl;
        }
    }


    //进行多项式的求解，得到对应的lambda．
    double lambda;
    if(SolverFourthOrderPolynomial(poly_coffi,lambda) == false)
    {
        std::cout <<"Polynomial Solve Failed"<<std::endl;
        return false;
    }

    //得到lambda之后，根据式24．
    Eigen::Matrix4d W;
    W.setZero();
    W.block(2,2,2,2) = Eigen::Matrix2d::Identity();

    //Eigen::Vector4d res = -(M + 2 *lambda * W).inverse().transpose() * g;
    Eigen::Vector4d res = -(M + 2 * lambda * W).inverse() * g;

    //转换成旋转矩阵
    double theta = std::atan2(res(3),res(2));
    deltaTrans << cos(theta),-sin(theta),res(0),
            sin(theta), cos(theta),res(1),
            0,          0,     1;
    return true;
}


Eigen::Vector2d IMLSICPMatcher::ComputeNormal(std::vector<Eigen::Vector2d> &nearPoints)
{
    Eigen::Vector2d normal;

    // compute mean
    Eigen::Vector2d mean;
    mean.setZero();
    for(int i=0; i<nearPoints.size(); i++){
        mean += nearPoints[i];
    }
    mean = mean / nearPoints.size();

    // compute covariance matrix
    Eigen::Matrix2d cov;
    cov.setZero();
    for(int i=0; i<nearPoints.size(); i++){
        Eigen::Vector2d pt = nearPoints[i] - mean;
        cov(0,0) += pt[0] * pt[0];
        cov(0,1) += pt[0] * pt[1];
        cov(1,0) += pt[1] * pt[0];
        cov(1,1) += pt[1] * pt[1];
    }
    cov = cov / nearPoints.size();

    // normal vector is the eigen vector with respect to the smallest eigen value
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix2d> eig(cov);
    Eigen::Vector2d eigValues = eig.eigenvalues();
    Eigen::Matrix2d eigVectors = eig.eigenvectors();
    normal = eigVectors.col(0);

    return normal;
}


/**
 * @brief IMLSICPMatcher::Match
 * @param finalResult
 * @param covariance
 * @return
 */
bool IMLSICPMatcher::Match(Eigen::Matrix3d& finalResult,
                           Eigen::Matrix3d& covariance)
{

    // if normal vectors are not set, first compute normal vectors of target(previous) point cloud
    if(m_isGetNormals == false)
    {
        // for each point in target point cloud, compute its normal vector
        m_targetPtCloudNormals.clear();
        for(int i = 0; i < m_targetPointCloud.size();i++)
        {
            Eigen::Vector2d xi = m_targetPointCloud[i];

            // take 20 nearest points as neighbors
            int K = 20;
            Eigen::VectorXi indices(K);
            Eigen::VectorXd dist2(K);
            int num = m_pTargetKDTree->knn(xi,indices,dist2,K,0.0,
                                           Nabo::NNSearchD::SORT_RESULTS | Nabo::NNSearchD::ALLOW_SELF_MATCH|
                                           Nabo::NNSearchD::TOUCH_STATISTICS,
                                           0.15);

            std::vector<Eigen::Vector2d> nearPoints;
            for(int ix = 0; ix < K;ix++)
            {
                if(dist2(ix) < std::numeric_limits<double>::infinity() &&
                        std::isinf(dist2(ix)) == false)
                {
                    nearPoints.push_back(m_targetKDTreeDataBase.col(indices(ix)));
                }
                else break;
            }

            Eigen::Vector2d normal;
            if(nearPoints.size() > 5)
            {
                normal = ComputeNormal(nearPoints);
            }
            else
            {
                normal(0) = normal(1) = std::numeric_limits<double>::infinity();
            }
            m_targetPtCloudNormals.push_back(normal);
        }
    }

    // set initial estimation
    Eigen::Matrix3d result;
    result.setIdentity();
    covariance.setIdentity();

    for(int i = 0; i < m_Iterations;i++)
    {
        // transform the source(current) point cloud arrcording to estimation
        std::vector<Eigen::Vector2d> in_cloud;
        for(int ix = 0; ix < m_sourcePointCloud.size();ix++)
        {
            Eigen::Vector3d origin_pt;
            origin_pt << m_sourcePointCloud[ix],1;

            Eigen::Vector3d now_pt = result * origin_pt;
            in_cloud.push_back(Eigen::Vector2d(now_pt(0),now_pt(1)));
        }

        // in_cloud: transformed source point cloud
        // ref_cloud: corresponding points which are projected from in_cloud to target point cloud surface
        std::vector<Eigen::Vector2d> ref_cloud;
        std::vector<Eigen::Vector2d> ref_normal;
        projSourcePtToSurface(in_cloud,
                              ref_cloud,
                              ref_normal);

        if(in_cloud.size() < 5 || ref_cloud.size() < 5)
        {
            std::cout <<"Not Enough Correspondence:"<<in_cloud.size()<<","<<ref_cloud.size()<<std::endl;
            std::cout <<"ICP Iterations Failed!!"<<std::endl;
            return false;
        }

        // compute euclidean motion between two laser frames, from source to target
        Eigen::Matrix3d deltaTrans;
        bool flag = SolveMotionEstimationProblem(in_cloud,
                                                 ref_cloud,
                                                 ref_normal,
                                                 deltaTrans);

        if(flag == false)
        {
            std::cout <<"ICP Iterations Failed!!!!"<<std::endl;
            return false;
        }

        // update transformation
        result = deltaTrans * result;

        // break condition
        double deltadist = std::sqrt( std::pow(deltaTrans(0,2),2) + std::pow(deltaTrans(1,2),2));
        double deltaAngle = std::atan2(deltaTrans(1,0),deltaTrans(0,0));

        if(deltadist < 0.001 && deltaAngle < (0.01/57.295))
        {
            break;
        }
    }

    finalResult = result;
    return true;
}


// find first non-zero real root of four order polynomial
bool IMLSICPMatcher::SolverFourthOrderPolynomial(Eigen::VectorXd&p_coffi,
                                                 double &lambda)
{
    Eigen::PolynomialSolver<double,4> polySolve(p_coffi);

    Eigen::Matrix<std::complex<double>,4,1,0,4,1> roots = polySolve.roots();

    bool isAssigned = false;
    double finalRoot = 0.0;

    //找到第一个非零实根--有可能不止一个，因为有优化空间．
    for(int i = 0; i < roots.size();i++)
    {
        //如果是虚根，则不要．
        if(roots(i).imag() != 0 )continue;

        //如果是非零实根，则选择．
        if(isAssigned == false ||  roots(i).real() > finalRoot)
        {
            isAssigned = true;
            finalRoot = roots(i).real();
        }
    }

    lambda = finalRoot;
    return isAssigned;
}

