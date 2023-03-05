/**
 * @file perception_node.cpp
 * @brief Process LiDAR pointcloud data
 * @author Adrian Sochaniwsky (sochania@mcmaster.ca)
 * @version 0.1
 * @date 2023-01-12
 *
 * @todo
 *      - Convert axis-aligned BB to OBB
 *      - create functions and clean up callback
 *      - create .hpp and .cpp to make easier reading
 *      - Use IPC by running this node in a container with the Ouster Node
 *
 * @copyright Copyright (c) 2023
 *
 */

#include "lidar_pipeline/perception_node.hpp"

#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>

void lidar_pipeline::PerceptionNode::cloud_callback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr recent_cloud)
{
    // RCLCPP_INFO(this->get_logger(), "Cloud service called; getting a PointCloud2 on topic");
    auto start = std::chrono::high_resolution_clock::now();

    /*
        * TRANSFORM PointCloud2 FROM CAMERA FRAME TO WORLD FRAME
        */
    geometry_msgs::msg::TransformStamped stransform;
    try
    {
        stransform = tf_buffer_->lookupTransform(world_frame, recent_cloud->header.frame_id,
                                                    tf2::TimePointZero, tf2::durationFromSec(3));
    }
    catch (const tf2::TransformException &ex)
    {
        RCLCPP_ERROR(this->get_logger(), "%s", ex.what());
    }

    sensor_msgs::msg::PointCloud2 transformed_cloud;
    pcl_ros::transformPointCloud(world_frame, stransform, *recent_cloud, transformed_cloud);

    /*
        * CONVERT PointCloud2 ROS->PCL
        */
    pcl::PointCloud<pcl::PointXYZI> cloud;
    pcl::fromROSMsg(transformed_cloud, cloud);

    /* ========================================
        * VOXEL GRID
        * ========================================*/
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_ptr(new pcl::PointCloud<pcl::PointXYZI>(cloud));
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_voxel_filtered(new pcl::PointCloud<pcl::PointXYZI>());
    pcl::VoxelGrid<pcl::PointXYZI> voxel_filter;
    voxel_filter.setInputCloud(cloud_ptr);
    voxel_filter.setLeafSize(voxel_leaf_size, voxel_leaf_size, voxel_leaf_size);
    voxel_filter.filter(*cloud_voxel_filtered);

    /* ========================================
        * CROPBOX
        * ========================================*/
    pcl::PointCloud<pcl::PointXYZI> xyz_filtered_cloud;
    pcl::CropBox<pcl::PointXYZI> crop;
    crop.setInputCloud(cloud_voxel_filtered);
    Eigen::Vector4f min_point = Eigen::Vector4f(x_filter_min, y_filter_min, z_filter_min, 0);
    Eigen::Vector4f max_point = Eigen::Vector4f(x_filter_max, y_filter_max, z_filter_max, 0);
    crop.setMin(min_point);
    crop.setMax(max_point);
    // crop.setKeepOrganized(true);
    crop.filter(xyz_filtered_cloud);

    /* ========================================
        * STATISTICAL OUTLIER REMOVAL
        * ========================================*/
    pcl::PointCloud<pcl::PointXYZI>::Ptr sor_input_cloud(new pcl::PointCloud<pcl::PointXYZI>(xyz_filtered_cloud));
    pcl::PointCloud<pcl::PointXYZI>::Ptr sor_cloud_filtered(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::StatisticalOutlierRemoval<pcl::PointXYZI> sor;
    sor.setInputCloud(sor_input_cloud);
    sor.setMeanK(50);
    sor.setStddevMulThresh(1.0);
    sor.filter(*sor_cloud_filtered);
    
    /* ========================================
        * PLANE SEGEMENTATION
        * ========================================*/
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_f(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_plane(new pcl::PointCloud<pcl::PointXYZI>());
    // Create the segmentation object for the planar model and set all the parameters
    pcl::SACSegmentation<pcl::PointXYZI> seg;
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setMaxIterations(plane_max_iter);
    seg.setDistanceThreshold(plane_dist_thresh);
    // Segment the largest planar component from the cropped cloud
    seg.setInputCloud(sor_cloud_filtered);
    seg.segment(*inliers, *coefficients);
    if (inliers->indices.size() == 0)
    {
        RCLCPP_WARN(this->get_logger(), "Could not estimate a planar model for the given dataset.");
    }

    // Extract the planar inliers from the input cloud
    pcl::ExtractIndices<pcl::PointXYZI> extract;
    extract.setInputCloud(sor_cloud_filtered);
    extract.setIndices(inliers);
    extract.setNegative(false);

    // Get the points associated with the planar surface
    extract.filter(*cloud_plane);
    // RCLCPP_INFO(this->get_logger(),
    //             "PointCloud2 representing the planar component: '%lu' data points.", cloud_plane->points.size());

    // Remove the planar inliers, extract the rest
    extract.setNegative(true);
    extract.filter(*cloud_f);

    /* ========================================
        * EUCLIDEAN CLUSTER EXTRACTION
        * ========================================*/
    // Creating the KdTree object for the search method of the extraction
    pcl::search::KdTree<pcl::PointXYZI>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZI>);
    *cloud_filtered = *cloud_f;
    tree->setInputCloud(cloud_filtered);

    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZI> ec;
    ec.setClusterTolerance(cluster_tol);
    ec.setMinClusterSize(cluster_min_size);
    ec.setMaxClusterSize(cluster_max_size);
    ec.setSearchMethod(tree);
    ec.setInputCloud(cloud_filtered);
    ec.extract(cluster_indices);


    std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> clusters;
    std::vector<vision_msgs::msg::BoundingBox3D> bboxes;
    CubePoints max_min_pts;
    
    for (const auto &cluster : cluster_indices)
    {
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_cluster(new pcl::PointCloud<pcl::PointXYZI>);

        // Compute MinMax points AABB
        Eigen::Vector4f minPt{}, maxPt{};
        pcl::getMinMax3D(*cloud_filtered, cluster, minPt, maxPt);

        max_min_pts.max_pts.push_back(maxPt);
        max_min_pts.min_pts.push_back(minPt);
        
        // Put each cluster into a vector
        for (const auto &idx : cluster.indices)
        {
            cloud_cluster->points.push_back((*cloud_filtered)[idx]);
        }
        cloud_cluster->width = cloud_cluster->points.size();
        cloud_cluster->height = 1;
        cloud_cluster->is_dense = true;

        // RCLCPP_INFO(this->get_logger(), "Cluster has '%lu' points", cloud_cluster->points.size());
        clusters.push_back(cloud_cluster);

        // Init and fill bboxes
        vision_msgs::msg::BoundingBox3D bb = getOrientedBoudingBox(*cloud_cluster);
        bboxes.push_back(bb);
    }
    
    /* ========================================
        * Compute Bounding Boxes
        * ========================================*/
    std::vector<geometry_msgs::msg::Point> line_list = minMax2lines(max_min_pts);

    /* ========================================
        * CONVERT PointCloud2 PCL->ROS, PUBLISH CLOUD
        * ========================================*/
    this->publishPointCloud(voxel_grid_pub_, *cloud_voxel_filtered);
    this->publishPointCloud(plane_pub_, *cloud_f);
    this->publishPointCloud(euclidean_pub_, *clusters[0]);
    this->publishPointCloud(stat_pub_, *sor_cloud_filtered);

    this->publish3DBBox(marker_pub_, line_list);
    this->publish3DBBoxOBB(marker_array_pub_, bboxes);

    auto stop = std::chrono::high_resolution_clock::now();
    auto t_ms = std::chrono::duration_cast<std::chrono::milliseconds>(stop - start);
    RCLCPP_INFO(get_logger(), "Time (msec): %ld", t_ms.count());
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<lidar_pipeline::PerceptionNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
