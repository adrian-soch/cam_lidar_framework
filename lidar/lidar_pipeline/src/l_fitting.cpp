#include "lidar_pipeline/l_fitting.hpp"
#include <cmath>
#include <Eigen/Core>
#include <vector>

using namespace std;

float recFitting::calc_closeness_criterion(Eigen::MatrixXf& cluster_2d_vc)
{
    float c1_max = cluster_2d_vc.row(0).maxCoeff();
    float c2_max = cluster_2d_vc.row(1).maxCoeff();
    float c1_min = cluster_2d_vc.row(0).minCoeff();
    float c2_min = cluster_2d_vc.row(1).minCoeff();

    Eigen::VectorXf arrayOfX = cluster_2d_vc.row(0);
    Eigen::VectorXf arrayOfY = cluster_2d_vc.row(1);

    Eigen::VectorXf a  = (c1_max * Eigen::VectorXf::Ones(cluster_2d_vc.cols())) - arrayOfX;
    Eigen::VectorXf b  = arrayOfX - Eigen::VectorXf::Ones(cluster_2d_vc.cols()) * c1_min;
    Eigen::VectorXf D1 = a.cwiseMin(b);

    Eigen::VectorXf c  = Eigen::VectorXf::Ones(cluster_2d_vc.cols()) * c2_max - arrayOfY;
    Eigen::VectorXf d  = arrayOfY - Eigen::VectorXf::Ones(cluster_2d_vc.cols()) * c2_min;
    Eigen::VectorXf D2 = c.cwiseMin(d);

    Eigen::VectorXf minDis = min_dist_of_closeness_crit * Eigen::VectorXf::Ones(cluster_2d_vc.cols());


    Eigen::VectorXf D3 = D1.cwiseMin(D2);
    Eigen::VectorXf D4 = D3.cwiseMax(minDis);
    Eigen::VectorXf D5 = D4.cwiseInverse();
    float distance     = D5.sum();

    return distance;
}

int recFitting::calc_boundingBox(float minp[2], Eigen::MatrixXf& cluster_3d)
{
    Eigen::Matrix2f minTrans;

    minTrans << cosf(minp[1]), sinf(minp[1]), -sinf(minp[1]), cosf(minp[1]);

    // x,y,z to x,y
    Eigen::MatrixXf cluster_2d = cluster_3d.topRows(2);
    // trans to object coordinate
    Eigen::MatrixXf cluster_final = minTrans * cluster_2d;

    // get the object border in object coordinate
    float c1_max = cluster_final.row(0).maxCoeff();
    float c2_max = cluster_final.row(1).maxCoeff();
    float c1_min = cluster_final.row(0).minCoeff();
    float c2_min = cluster_final.row(1).minCoeff();
    float c3_max = cluster_3d.row(2).maxCoeff();
    float c3_min = cluster_3d.row(2).minCoeff();

    Eigen::Vector2f centerPoint;
    centerPoint << (c1_max + c1_min) / 2.0f, (c2_max + c2_min) / 2.0f;

    // trans center point to vehicle coordinate
    centerPoint = minTrans.inverse() * centerPoint;

    lShapeReuslt tmpRlt;
    // calculate translation matrix and box size
    tmpRlt.width   = c2_max - c2_min;
    tmpRlt.length   = c1_max - c1_min;
    tmpRlt.height = c3_max - c3_min;
    tmpRlt.translation << centerPoint(0), centerPoint(1), (c3_max + c3_min) / 2.0f;

    // calculate ratation matrix
    // TBD: the sign is opposite to right hand coordinate system
    Eigen::Matrix3f r_matrix = Eigen::Matrix3f::Identity();
    r_matrix << cosf(-minp[1]), sinf(-minp[1]), 0,
        -sinf(-minp[1]), cosf(-minp[1]), 0,
        0, 0, 1;
    tmpRlt.rotation = r_matrix;

    shapeRlt = tmpRlt;
    return 0;
} // recFitting::calc_boundingBox

void recFitting::rectangle_search(Eigen::MatrixXf& cluster_3d)
{
    float dtheta  = dtheta_deg_for_serarch * (M_PI / 180.0F);
    float minp[2] = { -FLT_MAX, 0.0 };

    // x,y,z to x,y
    Eigen::MatrixXf cluster_2d = cluster_3d.topRows(2);

    for(float theta = 0.0F; theta <= M_PI / 2.0F; theta += dtheta) {
        Eigen::Matrix2f transM;
        transM << cosf(theta), sinf(theta), -sinf(theta), cosf(theta);

        Eigen::MatrixXf cluster_vc = transM * cluster_2d; // translate to object coordinate

        float cost = calc_closeness_criterion(cluster_vc);

        if(minp[0] < cost) {
            minp[0] = cost;
            minp[1] = theta;
        }
    }

    calc_boundingBox(minp, cluster_3d);
}

void recFitting::fitting(const pcl::PointCloud<pcl::PointXYZI>& cloudPoints)
{
    Eigen::MatrixXf cluster_3d(3, cloudPoints.points.size());
    int index = 0;

    for(size_t pit=0; pit < cloudPoints.points.size(); pit++) {
        cluster_3d(0, index) = cloudPoints.points[pit].x;
        cluster_3d(1, index) = cloudPoints.points[pit].y;
        cluster_3d(2, index) = cloudPoints.points[pit].z;
        ++index;
    }
    rectangle_search(cluster_3d);

} // recFitting::fitting
