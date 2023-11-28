/**
 * @file point_cloud_utils.hpp
 * @author Adrian Sochaniwsky (sochania@mcmaster.ca)
 * @brief Utility functions for point cloud operations
 */

#ifndef POINT_CLOUD_UTILITIES_HPP_
#define POINT_CLOUD_UTILITIES_HPP_

#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>

#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/conditional_euclidean_clustering.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/extract_polygonal_prism_data.h>
#include <pcl/segmentation/region_growing.h>
#include <pcl/segmentation/sac_segmentation.h>

// Application specific headers
#include "optics.hpp"

template<typename PointT>
class Operations {
public:
    using PointCloud         = pcl::PointCloud<PointT>;
    using PointCloudPtr      = typename PointCloud::Ptr;
    using PointCloudConstPtr = typename PointCloud::ConstPtr;

    using Tree    = pcl::search::KdTree<PointT>;
    using TreePtr = typename Tree::Ptr;

    /**
     * @brief Perform voxel grid filtering on a point cloud
     *
     * @tparam PointT: Type of point cloud (ex. XYZ, XYZI)
     * @param cloud_ptr: Pointer to a PCL cloud object
     * @param voxel_leaf_size: size in meters of the voxel grid
     */
    void voxel_grid_filter(PointCloudPtr cloud_ptr, float voxel_leaf_size_x, float voxel_leaf_size_y,
      float voxel_leaf_size_z)
    {
        pcl::VoxelGrid<PointT> voxel_filter;

        voxel_filter.setInputCloud(cloud_ptr);
        voxel_filter.setLeafSize(voxel_leaf_size_x, voxel_leaf_size_y, voxel_leaf_size_z);
        voxel_filter.filter(*cloud_ptr);
    }

    /**
     * @brief Crops points outside of the cube defined by the x/y/z params
     *          box is centered about the point (0,0,0)
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
      float x_max, float y_max, float z_max, bool keep_organized)
    {
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
     * @brief Crop everything outside of the provided rectangular prism
     *
     * @param cloud_ptr
     * @param transform
     * @param box_width float
     * @param box_length float
     * @param box_height float
     */
    void prism_segmentation(PointCloudPtr cloud_ptr, Eigen::Affine3f &transform, float box_width, float box_length,
      float box_height, bool inverse_selection=false)
    {
        PointCloudPtr pick_surface_cloud_ptr(new PointCloud());
        pcl::PointIndices::Ptr pt_inliers(new pcl::PointIndices());
        pcl::ExtractIndices<PointT> extract_ind;

        pcl::ExtractPolygonalPrismData<PointT> prism;

        prism.setInputCloud(cloud_ptr);

        pick_surface_cloud_ptr->width  = 5;
        pick_surface_cloud_ptr->height = 1;
        pick_surface_cloud_ptr->points.resize(5);

        pick_surface_cloud_ptr->points[0].x = 0.5f * box_width;
        pick_surface_cloud_ptr->points[0].y = 0.5f * box_length;
        pick_surface_cloud_ptr->points[0].z = 0.0;

        pick_surface_cloud_ptr->points[1].x = -0.5f * box_width;
        pick_surface_cloud_ptr->points[1].y = 0.5f * box_length;
        pick_surface_cloud_ptr->points[1].z = 0;

        pick_surface_cloud_ptr->points[2].x = -0.5f * box_width;
        pick_surface_cloud_ptr->points[2].y = -0.5f * box_length;
        pick_surface_cloud_ptr->points[2].z = 0.0;

        pick_surface_cloud_ptr->points[3].x = 0.5f * box_width;
        pick_surface_cloud_ptr->points[3].y = -0.5f * box_length;
        pick_surface_cloud_ptr->points[3].z = 0;

        pick_surface_cloud_ptr->points[4].x = 0.5f * box_width;
        pick_surface_cloud_ptr->points[4].y = 0.5f * box_length;
        pick_surface_cloud_ptr->points[4].z = 0;


        pcl::transformPointCloud(*pick_surface_cloud_ptr, *pick_surface_cloud_ptr, transform);
        prism.setInputPlanarHull(pick_surface_cloud_ptr);
        prism.setHeightLimits(-box_height, box_height);
        prism.segment(*pt_inliers);

        extract_ind.setInputCloud(cloud_ptr);
        extract_ind.setIndices(pt_inliers);
        extract_ind.setNegative(inverse_selection);
        extract_ind.setKeepOrganized(true);
        extract_ind.filter(*cloud_ptr);
    } // prism_segmentation

    /**
     * @brief Remove statistical outliers
     *
     * @param cloud_ptr
     * @param num_neighbours
     * @param stddev_mult
     */
    void stats_outlier_removal(PointCloudPtr cloud_ptr, int num_neighbours, float stddev_mult)
    {
        pcl::StatisticalOutlierRemoval<PointT> sor;

        sor.setInputCloud(cloud_ptr);
        sor.setMeanK(num_neighbours);
        sor.setStddevMulThresh(stddev_mult);
        sor.filter(*cloud_ptr);
    }

    /**
     * @brief Estimate a plane with RANSAC, then remove points on the plane
     *
     * @param cloud_ptr
     * @param max_iterations: Max number of RANSAC itersations
     * @param dist_thresh   RANSAC threshold to add points to the consensus set
     * @return int:         0 for success, -1 if fails to find plane
     */
    int ground_plane_removal(PointCloudPtr cloud_ptr, int max_iterations, double dist_thresh)
    {
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
        if(inliers->indices.size() == 0)
            return -1;

        // Extract the planar inliers from the input cloud
        pcl::ExtractIndices<PointT> extract;
        extract.setInputCloud(cloud_ptr);
        extract.setIndices(inliers);
        extract.setNegative(false);
        extract.setKeepOrganized(true);

        // Get the points associated with the planar surface
        extract.filter(*cloud_plane);

        // Remove the planar inliers, extract the rest
        extract.setNegative(true);
        extract.filter(*cloud_ptr);

        return 0;
    } // ground_plane_removal

    /**
     * @brief Perform euclidean clustering on an given point cloud
     *
     * @param cloud_ptr
     * @param cluster_indices
     * @param cluster_tol
     * @param cluster_min_size
     * @param cluster_max_size
     */
    void euclidean_clustering(PointCloudPtr cloud_ptr, std::vector<pcl::PointIndices> &cluster_indices,
      double cluster_tol, int cluster_min_size, int cluster_max_size)
    {
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

    void conditional_euclidian_clustering(PointCloudPtr cloud_ptr)
    { }

    /**
     * @brief
     *
     * @param cloud_ptr
     * @param min_cluster_size
     * @param max_cluster_size
     * @param num_neighbours
     * @param smoothness_threshold
     * @param curvature_threshold
     * @param cluster_indices
     */
    void region_growing_clustering(PointCloudPtr cloud_ptr, int min_cluster_size, int max_cluster_size,
      int num_neighbours, float smoothness_threshold, float curvature_threshold,
      std::vector<pcl::PointIndices> &cluster_indices)
    {
        TreePtr tree(new Tree);

        pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal> );
        pcl::NormalEstimation<PointT, pcl::Normal> normal_estimator;

        normal_estimator.setSearchMethod(tree);
        normal_estimator.setInputCloud(cloud_ptr);
        normal_estimator.setKSearch(50);
        normal_estimator.compute(*normals);

        pcl::IndicesPtr indices(new std::vector<int> );
        pcl::removeNaNFromPointCloud(*cloud_ptr, *indices);

        pcl::RegionGrowing<PointT, pcl::Normal> reg;
        reg.setMinClusterSize(min_cluster_size);
        reg.setMaxClusterSize(max_cluster_size);
        reg.setSearchMethod(tree);
        reg.setNumberOfNeighbours(num_neighbours);
        reg.setInputCloud(cloud_ptr);
        reg.setIndices(indices);
        reg.setInputNormals(normals);
        reg.setSmoothnessThreshold(smoothness_threshold / 180.0 * M_PI);
        reg.setCurvatureThreshold(curvature_threshold);
        reg.extract(cluster_indices);
    }

    /**
     * @brief
     *
     * @param cloud_ptr
     * @param min_cluster_size
     * @param max_cluster_size
     * @param num_neighbours
     * @param smoothness_threshold
     * @param curvature_threshold
     * @param cluster_indices
     * @param colored_cloud
     */
    void region_growing_clustering(PointCloudPtr cloud_ptr, int min_cluster_size, int max_cluster_size,
      int num_neighbours, float smoothness_threshold, float curvature_threshold,
      std::vector<pcl::PointIndices> &cluster_indices,
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr colored_cloud)
    {
        TreePtr tree(new Tree);

        pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal> );
        pcl::NormalEstimation<PointT, pcl::Normal> normal_estimator;

        normal_estimator.setSearchMethod(tree);
        normal_estimator.setInputCloud(cloud_ptr);
        normal_estimator.setKSearch(50);
        normal_estimator.compute(*normals);

        pcl::IndicesPtr indices(new std::vector<int> );
        pcl::removeNaNFromPointCloud(*cloud_ptr, *indices);

        pcl::RegionGrowing<PointT, pcl::Normal> reg;
        reg.setMinClusterSize(min_cluster_size);
        reg.setMaxClusterSize(max_cluster_size);
        reg.setSearchMethod(tree);
        reg.setNumberOfNeighbours(num_neighbours);
        reg.setInputCloud(cloud_ptr);
        reg.setIndices(indices);
        reg.setInputNormals(normals);
        reg.setSmoothnessThreshold(smoothness_threshold / 180.0 * M_PI);
        reg.setCurvatureThreshold(curvature_threshold);
        reg.extract(cluster_indices);

        colored_cloud = reg.getColoredCloud();
    }

    typedef pcl::PointXYZI PointTypeIO;
    typedef pcl::PointXYZINormal PointTypeFull;
    static bool enforceIntensitySimilarity(const PointTypeFull& point_a, const PointTypeFull& point_b,
      float squared_distance)
    {
        if(std::abs(point_a.intensity - point_b.intensity) < 5.0f)
            return (true);
        else
            return (false);
    }

    static bool enforceCurvatureOrIntensitySimilarity(const PointTypeFull& point_a, const PointTypeFull& point_b,
      float squared_distance)
    {
        Eigen::Map<const Eigen::Vector3f> point_a_normal = point_a.getNormalVector3fMap(),
          point_b_normal = point_b.getNormalVector3fMap();

        if(std::abs(point_a.intensity - point_b.intensity) < 5.0f)
            return (true);

        if(std::abs(point_a_normal.dot(point_b_normal)) < 0.05)
            return (true);

        return (false);
    }

    static bool customRegionGrowing(const PointTypeFull& point_a, const PointTypeFull& point_b, float squared_distance)
    {
        Eigen::Map<const Eigen::Vector3f> point_a_normal = point_a.getNormalVector3fMap(),
          point_b_normal = point_b.getNormalVector3fMap();

        if(squared_distance < 10000) {
            if(std::abs(point_a.intensity - point_b.intensity) < 800.0f)
                return (true);

            if(std::abs(point_a_normal.dot(point_b_normal)) < 6.0)
                return (true);
        } else {
            if(std::abs(point_a.intensity - point_b.intensity) < 300.0f)
                return (true);
        }
        return (false);
    }

    void conditional_euclidean_clustering(PointCloudPtr cloud_ptr, std::vector<pcl::PointIndices> &cluster_indices)
    {
        // Data containers used
        pcl::PointCloud<PointTypeFull>::Ptr cloud_with_normals(new pcl::PointCloud<PointTypeFull> );
        pcl::IndicesClustersPtr clusters(new pcl::IndicesClusters), small_clusters(new pcl::IndicesClusters),
        large_clusters(new pcl::IndicesClusters);
        TreePtr search_tree(new Tree);

        pcl::NormalEstimation<PointT, PointTypeFull> ne;

        ne.setInputCloud(cloud_ptr);
        ne.setSearchMethod(search_tree);
        ne.setRadiusSearch(300.0);
        ne.compute(*cloud_with_normals);

        // Set up a Conditional Euclidean Clustering class
        pcl::ConditionalEuclideanClustering<PointTypeFull> cec(true);
        cec.setInputCloud(cloud_with_normals);
        cec.setConditionFunction(&customRegionGrowing);
        cec.setClusterTolerance(500.0);
        cec.setMinClusterSize(cloud_with_normals->size() / 1000);
        cec.setMaxClusterSize(cloud_with_normals->size() / 5);
        cec.segment(cluster_indices);
        cec.getRemovedClusters(small_clusters, large_clusters);
    }

    void optics_clustering(PointCloudPtr cloud_ptr, std::vector<pcl::PointIndices> &cluster_indices, int min_pts,
      float reachability_thresh)
    {
        std::vector<pcl::PointIndicesPtr> indices_ptr_vector; // your input vector

        Optics::optics<PointT>(cloud_ptr, min_pts, reachability_thresh, indices_ptr_vector);

        for(auto &indices_ptr : indices_ptr_vector) {            // loop over each element
            pcl::PointIndices indices = std::move(*indices_ptr); // access and move the value pointed by the shared pointer
            cluster_indices.push_back(indices);                  // add the value to the output vector
        }
    }

    void warp_density(PointCloudPtr cloud_ptr, float max_range = 120.0)
    {
        for(size_t idx = 0; idx < cloud_ptr->points.size(); ++idx) {
            float x = cloud_ptr->points[idx].x;
            float y = cloud_ptr->points[idx].y;
            // float z = cloud_ptr->points[idx].z;

            float max_detection_range = max_range - 30.0;

            float range = sqrt(pow(x, 2.0) + pow(y, 2.0));
            float value = std::clamp(range / max_detection_range, 0.0f, 1.0f);
            float alpha = tan(value * 1.56999) / tan(1.5699) * max_range;

            cloud_ptr->points[idx].x = (range - alpha) * cos(atan2f(y, x));
            cloud_ptr->points[idx].y = (range - alpha) * sin(atan2f(y, x));
        }
    }
};

#endif // ifndef POINT_CLOUD_UTILITIES_HPP_
