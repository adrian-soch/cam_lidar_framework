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


template<typename PointT>
class Operations {
public:
    using PointCloud = pcl::PointCloud<PointT>;
    using PointCloudPtr = typename PointCloud::Ptr;
    using PointCloudConstPtr = typename PointCloud::ConstPtr;

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

};

#endif