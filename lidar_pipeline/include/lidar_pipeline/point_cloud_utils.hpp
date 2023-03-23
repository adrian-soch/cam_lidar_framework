/**
 * @file point_cloud_utils.hpp
 * @author Adrian Sochaniwsky (sochania@mcmaster.ca)
 * @brief Utility functions for point cloud operations
 * @version 0.1
 * @date 2023-03-23
 * 
 * @copyright Copyright (c) 2023
 * 
 */

#ifndef POINT_CLOUD_UTILITIES_HPP_
#define POINT_CLOUD_UTILITIES_HPP_

#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/statistical_outlier_removal.h>

#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>

#include <pcl/common/common.h>

template<typename PointT>
class Operations {
public:
    using PointCloud = pcl::PointCloud<PointT>;
    using PointCloudPtr = typename PointCloud::Ptr;
    using PointCloudConstPtr = typename PointCloud::ConstPtr;

    using Tree = pcl::search::KdTree<PointT>;
    using TreePtr = typename Tree::Ptr;

    /**
     * @brief 
     * 
     * @tparam PointT 
     * @param cloud_ptr 
     * @param voxel_leaf_size 
     */
    void voxel_grid_filter(PointCloudPtr cloud_ptr, float voxel_leaf_size) {

        pcl::VoxelGrid<PointT> voxel_filter;
        voxel_filter.setInputCloud(cloud_ptr);
        voxel_filter.setLeafSize(voxel_leaf_size, voxel_leaf_size, voxel_leaf_size);
        voxel_filter.filter(*cloud_ptr);
    }

    /**
     * @brief 
     * 
     * @tparam PointT
     * @param cloud_ptr
     * @param x_min
     * @param y_min
     * @param z_min
     * @param x_max
     * @param y_max
     * @param z_max
     * @param keep_organized
     */
    void crop_box_filter(PointCloudPtr cloud_ptr,
        float x_min, float y_min, float z_min,
        float x_max, float y_max, float z_max, bool keep_organized) {
        
        pcl::CropBox<PointT> crop;
        crop.setInputCloud(cloud_ptr);
        Eigen::Vector4f min_point = Eigen::Vector4f(x_min, y_min, z_min, 0);
        Eigen::Vector4f max_point = Eigen::Vector4f(x_max, y_max, z_max, 0);
        crop.setMin(min_point);
        crop.setMax(max_point);
        crop.setKeepOrganized(keep_organized);
        crop.filter(*cloud_ptr);
    }

    /**
     * @brief 
     * 
     * @param cloud_ptr 
     * @param mean 
     * @param stddev_mult 
     */
    void stats_outlier_removal(PointCloudPtr cloud_ptr, int mean, float stddev_mult) {
        pcl::StatisticalOutlierRemoval<PointT> sor;
        sor.setInputCloud(cloud_ptr);
        sor.setMeanK(mean);
        sor.setStddevMulThresh(stddev_mult);
        sor.filter(*cloud_ptr);
    }

    int ground_plane_removal(PointCloudPtr cloud_ptr, int max_iterations, double dist_thresh) {

        PointCloudPtr cloud_plane(new PointCloud());

        pcl::SACSegmentation<PointT> seg;
        pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
        pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
        seg.setOptimizeCoefficients(true);
        seg.setModelType(pcl::SACMODEL_PLANE);
        seg.setMethodType(pcl::SAC_RANSAC);
        seg.setMaxIterations(max_iterations);
        seg.setDistanceThreshold(dist_thresh);

        // Segment the largest planar component from the cropped cloud
        seg.setInputCloud(cloud_ptr);
        seg.segment(*inliers, *coefficients);
        if (inliers->indices.size() == 0)
            return -1;
        
        // Extract the planar inliers from the input cloud
        pcl::ExtractIndices<PointT> extract;
        extract.setInputCloud(cloud_ptr);
        extract.setIndices(inliers);
        extract.setNegative(false);

        // Get the points associated with the planar surface
        extract.filter(*cloud_plane);

        // Remove the planar inliers, extract the rest
        extract.setNegative(true);
        extract.filter(*cloud_ptr);

        return 0;
    }

    void euclidean_clustering(PointCloudPtr cloud_ptr, std::vector<pcl::PointIndices> &cluster_indices,
        double cluster_tol, int cluster_min_size, int cluster_max_size) {

        TreePtr tree(new Tree);
        tree->setInputCloud(cloud_ptr);

        pcl::EuclideanClusterExtraction<PointT> ec;
        ec.setClusterTolerance(cluster_tol);
        ec.setMinClusterSize(cluster_min_size);
        ec.setMaxClusterSize(cluster_max_size);
        ec.setSearchMethod(tree);
        ec.setInputCloud(cloud_ptr);
        ec.extract(cluster_indices);
    }

};

#endif