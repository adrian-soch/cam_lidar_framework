#ifndef L_FITTING_HPP
#define L_FITTING_HPP

#include <Eigen/Core>
#include <pcl/filters/extract_indices.h>
#include <pcl/point_types.h>
#include <vector>

struct lShapeReuslt {
    Eigen::Vector3f    translation;
    Eigen::Quaternionf rotation;
    float              width;
    float              height;
    float              length;
};

class recFitting {
public:
    void
    fitting(const pcl::PointCloud<pcl::PointXYZI>& cloudPoints);
    lShapeReuslt shapeRlt;

private:
    float
    calc_closeness_criterion(Eigen::MatrixXf& cluster_2d_vc);
    int
    calc_boundingBox(float minp[2], Eigen::MatrixXf& cluster_3d);
    void
    rectangle_search(Eigen::MatrixXf& cluster_2d);


private:
    const float R0 = 3.0F;                          // [m] range segmentation param
    const float Rd = 0.001F;                        // [m] range segmentation param
    const float dtheta_deg_for_serarch     = 1.0F;  // [deg]
    const float min_dist_of_closeness_crit = 0.01F; // [m]
};

#endif // L_FITTING_HPP
