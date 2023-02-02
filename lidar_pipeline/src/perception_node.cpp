/**
 * @file perception_node.cpp
 * @brief Process LiDAR pointcloud data
 * @author Adrian Sochaniwsky (sochania@mcmaster.ca)
 * @version 0.1
 * @date 2023-01-12
 *
 * @todo
 *      - if structured cloud is maintained can we allocate all memory at compile time?
 *      - paralellize the median computation
 *      - downsampling that maintains structuredness (voxel does not)
 *      - create functions and clean up callback
 *      - create .hpp and .cpp to make easier reading
 *      - Use IPC by running this node in a container with the Ouster Node
 *
 * @copyright Copyright (c) 2023
 *
 */

#include <iostream> 

#include "rclcpp/rclcpp.hpp"
#include <rclcpp/qos.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <tf2/transform_datatypes.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>

#include <pcl_ros/transforms.hpp>
#include <pcl_conversions/pcl_conversions.h>

#include <pcl/common/common.h>
#include <pcl/common/io.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>


#include <tf2/convert.h>
#include <tf2_eigen/tf2_eigen.h>
#include <tf2/LinearMath/Quaternion.h>

float compute_median(std::vector<float> &vec)
{
    auto size = vec.size();

    if (size <= 0) {return nanf("");}

    std::sort(vec.begin(), vec.end());
    if (size % 2 == 0) {
      return (vec[size / 2-1] + vec[size / 2]) / 2;
    }
    else {
      return vec[size / 2];
    }
}

class PerceptionNode : public rclcpp::Node
{
public:
    PerceptionNode()
        : Node("perception_node", rclcpp::NodeOptions()
                                      .allow_undeclared_parameters(true)
                                      .automatically_declare_parameters_from_overrides(true))
    {
        /*
         * SET UP PUBLISHERS
         */
        RCLCPP_INFO(this->get_logger(), "Setting up publishers");

        voxel_grid_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("voxel_cluster", 1);
        passthrough_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("passthrough_cluster", 1);
        plane_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("plane_cluster", 1);
        euclidean_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("euclidean_cluster", 1);
        stat_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("stat_cluster", 1);
        polygon_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("polygon_cluster", 1);
        marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("markers", 1);

        /*
         * SET UP PARAMETERS (COULD BE INPUT FROM LAUNCH FILE/TERMINAL)
         */
        rclcpp::Parameter cloud_topic_param, world_frame_param, camera_frame_param, voxel_leaf_size_param,
            x_filter_min_param, x_filter_max_param, y_filter_min_param, y_filter_max_param, z_filter_min_param,
            z_filter_max_param, plane_max_iter_param, plane_dist_thresh_param, cluster_tol_param,
            cluster_min_size_param, cluster_max_size_param, median_filter_len_param;

        RCLCPP_INFO(this->get_logger(), "Getting parameters");

        this->get_parameter_or("cloud_topic", cloud_topic_param, rclcpp::Parameter("", "/points"));
        this->get_parameter_or("world_frame", world_frame_param, rclcpp::Parameter("", "laser_data_frame"));
        this->get_parameter_or("camera_frame", camera_frame_param, rclcpp::Parameter("", "laser_data_frame"));
        this->get_parameter_or("voxel_leaf_size", voxel_leaf_size_param, rclcpp::Parameter("", 0.25));
        this->get_parameter_or("x_filter_min", x_filter_min_param, rclcpp::Parameter("", 1.0));
        this->get_parameter_or("x_filter_max", x_filter_max_param, rclcpp::Parameter("", 120.0));
        this->get_parameter_or("y_filter_min", y_filter_min_param, rclcpp::Parameter("", -25.0));
        this->get_parameter_or("y_filter_max", y_filter_max_param, rclcpp::Parameter("", 10.0));
        this->get_parameter_or("z_filter_min", z_filter_min_param, rclcpp::Parameter("", -15.0));
        this->get_parameter_or("z_filter_max", z_filter_max_param, rclcpp::Parameter("", 15.0));
        this->get_parameter_or("plane_max_iterations", plane_max_iter_param, rclcpp::Parameter("", 100));
        this->get_parameter_or("plane_distance_threshold", plane_dist_thresh_param, rclcpp::Parameter("", 0.4));
        this->get_parameter_or("cluster_tolerance", cluster_tol_param, rclcpp::Parameter("", 1.5));
        this->get_parameter_or("cluster_min_size", cluster_min_size_param, rclcpp::Parameter("", 3));
        this->get_parameter_or("cluster_max_size", cluster_max_size_param, rclcpp::Parameter("", 1500));
        this->get_parameter_or("median_filter_len", median_filter_len_param, rclcpp::Parameter("", 31));

        cloud_topic = cloud_topic_param.as_string();
        world_frame = world_frame_param.as_string();
        camera_frame = camera_frame_param.as_string();
        voxel_leaf_size = float(voxel_leaf_size_param.as_double());
        x_filter_min = x_filter_min_param.as_double();
        x_filter_max = x_filter_max_param.as_double();
        y_filter_min = y_filter_min_param.as_double();
        y_filter_max = y_filter_max_param.as_double();
        z_filter_min = z_filter_min_param.as_double();
        z_filter_max = z_filter_max_param.as_double();
        plane_max_iter = plane_max_iter_param.as_int();
        plane_dist_thresh = plane_dist_thresh_param.as_double();
        cluster_tol = cluster_tol_param.as_double();
        cluster_min_size = cluster_min_size_param.as_int();
        cluster_max_size = cluster_max_size_param.as_int();
        median_filter_len = median_filter_len_param.as_int();

        /*
         * SET UP SUBSCRIBER
         */
        RCLCPP_INFO(this->get_logger(), "Setting up subscriber");

        cloud_subscriber_ =
            this->create_subscription<sensor_msgs::msg::PointCloud2>(
                cloud_topic, 1, std::bind(&PerceptionNode::cloud_callback, this, std::placeholders::_1));

        /*
         * SET UP TF
         */
        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
        br = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    }

private:
    struct CubePoints{
        std::vector<Eigen::Vector4f> max_pts;
        std::vector<Eigen::Vector4f> min_pts;
    };

    struct CubePointsPCL{
        std::vector<Eigen::Vector3f> max_pts;
        std::vector<Eigen::Vector3f> min_pts;
    };

    enum Axis {X, Y, Z};

    void cloud_callback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr recent_cloud)
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
        // pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_voxel_filtered(new pcl::PointCloud<pcl::PointXYZI>());
        // pcl::VoxelGrid<pcl::PointXYZI> voxel_filter;
        // voxel_filter.setInputCloud(cloud_ptr);
        // voxel_filter.setLeafSize(voxel_leaf_size, voxel_leaf_size, voxel_leaf_size);
        // voxel_filter.filter(*cloud_voxel_filtered);

        /* ========================================
         * CROPBOX
         * ========================================*/
        pcl::PointCloud<pcl::PointXYZI>::Ptr xyz_filtered_cloud_ptr(new pcl::PointCloud<pcl::PointXYZI>);
        // pcl::PointCloud<pcl::PointXYZI> xyz_filtered_cloud;
        pcl::CropBox<pcl::PointXYZI> crop;
        crop.setInputCloud(cloud_ptr);
        Eigen::Vector4f min_point = Eigen::Vector4f(x_filter_min, y_filter_min, z_filter_min, 0);
        Eigen::Vector4f max_point = Eigen::Vector4f(x_filter_max, y_filter_max, z_filter_max, 0);
        crop.setMin(min_point);
        crop.setMax(max_point);
        crop.setKeepOrganized(true);
        crop.filter(*xyz_filtered_cloud_ptr);

        /* ========================================
         * Temporal Median Filter (compute the static background)
         * ========================================*/
        // pcl::PointCloud<pcl::PointXYZI>::Ptr med_input_cloud(new pcl::PointCloud<pcl::PointXYZI>(xyz_filtered_cloud));
        static pcl::PointCloud<pcl::PointXYZI>::Ptr med_res(new pcl::PointCloud<pcl::PointXYZI>);
        static std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> buffer;
        if (buffer.size() >= median_filter_len) {
            buffer.erase(buffer.begin());
        }
        buffer.push_back(xyz_filtered_cloud_ptr);

        std::cout << "Buffer size: " << buffer.size() << std::endl;

        pcl::copyPointCloud(*xyz_filtered_cloud_ptr, *med_res);

        // std::cout << "Med w: " << med_res->width << " Med h: " << med_res->height << std::endl;
        // std::cout << "xyz w: " << xyz_filtered_cloud_ptr->width << " xyz h: " << xyz_filtered_cloud_ptr->height << std::endl;

        if (buffer.size() >= median_filter_len) {
            // Compute median value of z for each x,y position
            //      for each cloud in the buffer
            // CANDIDATE FOR MULTITHREADING?
            for(size_t x=0; x <  xyz_filtered_cloud_ptr->height; ++x) {
                for(size_t y=0; y < xyz_filtered_cloud_ptr->width; ++y) {

                    std::vector<float> z_vec;
                    for(auto cloud:buffer) {

                        auto value = cloud->at(y,x).z;

                        if(std::isnan(value)) {continue;}
                        z_vec.push_back(value);
                    }
                    auto med = compute_median(z_vec);
                    med_res->at(y,x).z = med;

                    // std::cout << "x: " << x << " y: " << y << std::endl;
                    // std::cout << "\nMed: " << med << std::endl;
                }
            }
        }

        /* ========================================
         * STATISTICAL OUTLIER REMOVAL
         * ========================================*/
        // pcl::PointCloud<pcl::PointXYZI>::Ptr sor_cloud_filtered(new pcl::PointCloud<pcl::PointXYZI>);
        pcl::PointCloud<pcl::PointXYZI> sor_cloud_filtered;
        // pcl::StatisticalOutlierRemoval<pcl::PointXYZI> sor;
        // sor.setInputCloud(med_res);
        // sor.setMeanK(50);
        // sor.setStddevMulThresh(1.0);
        // sor.setKeepOrganized(false);
        // sor.filter(sor_cloud_filtered);
        
        // std::cout << "Post stat outlier height: " << sor_cloud_filtered.height << std::endl;
        // std::cout << "Post stat outlier size: " << sor_cloud_filtered.size() << std::endl;

        auto stop1 = std::chrono::high_resolution_clock::now();
        auto t_ms1 = std::chrono::duration_cast<std::chrono::milliseconds>(stop1 - start);
        RCLCPP_INFO(get_logger(), "Filt time (msec): %ld", t_ms1.count());




        this->publishPointCloud(stat_pub_, *med_res);



        return;


        /* ========================================
         * PLANE SEGEMENTATION
         * ========================================*/
        pcl::PointCloud<pcl::PointXYZI>::Ptr sor_cloud_filtered_ptr(new pcl::PointCloud<pcl::PointXYZI>(sor_cloud_filtered));
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
        seg.setInputCloud(sor_cloud_filtered_ptr);
        seg.segment(*inliers, *coefficients);
        if (inliers->indices.size() == 0)
        {
            RCLCPP_WARN(this->get_logger(), "Could not estimate a planar model for the given dataset.");
        }

        // Extract the planar inliers from the input cloud
        pcl::ExtractIndices<pcl::PointXYZI> extract;
        extract.setInputCloud(sor_cloud_filtered_ptr);
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
        CubePoints max_min_pts;
        CubePointsPCL max_min_pts2;
        
        for (const auto &cluster : cluster_indices)
        {
            pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_cluster(new pcl::PointCloud<pcl::PointXYZI>);

            // Compute MinMax points ABB
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

            // Oriented Bounding Box (OBB)
            // http://codextechnicanum.blogspot.com/2015/04/find-minimum-oriented-bounding-box-of.html

            Eigen::Vector4f pcaCentroid;
            pcl::compute3DCentroid(*cloud_cluster, pcaCentroid);

            Eigen::Matrix3f covariance;
            pcl::computeCovarianceMatrixNormalized(*cloud_cluster, pcaCentroid, covariance);
            Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> eigen_solver(covariance, Eigen::ComputeEigenvectors);
            Eigen::Matrix3f eigenVectorsPCA = eigen_solver.eigenvectors();
            eigenVectorsPCA.col(2) = eigenVectorsPCA.col(0).cross(eigenVectorsPCA.col(1));

            // Transform the original cloud to the origin where the principal components correspond to the axes.
            Eigen::Matrix4f projectionTransform(Eigen::Matrix4f::Identity());
            projectionTransform.block<3,3>(0,0) = eigenVectorsPCA.transpose();
            projectionTransform.block<3,1>(0,3) = -1.f * (projectionTransform.block<3,3>(0,0) * pcaCentroid.head<3>());
            pcl::PointCloud<pcl::PointXYZI>::Ptr cloudPointsProjected (new pcl::PointCloud<pcl::PointXYZI>);
            pcl::transformPointCloud(*cloud_cluster, *cloudPointsProjected, projectionTransform);

            // Get the minimum and maximum points of the transformed cloud.
            pcl::PointXYZI minPoint, maxPoint;
            pcl::getMinMax3D(*cloudPointsProjected, minPoint, maxPoint);
            const Eigen::Vector3f meanDiagonal = 0.5f*(maxPoint.getVector3fMap() + minPoint.getVector3fMap());

            // Final transform
            const Eigen::Quaternionf bboxQuat(eigenVectorsPCA);
            bboxQuat.norm();
            const Eigen::Vector3f bboxTrans = eigenVectorsPCA * meanDiagonal + pcaCentroid.head<3>();

            auto maxp = bboxQuat*(maxPoint.getVector3fMap() + bboxTrans);
            auto minp = bboxQuat*(minPoint.getVector3fMap() + bboxTrans);
            max_min_pts2.max_pts.push_back(maxp);
            max_min_pts2.min_pts.push_back(minp);
        }
        
        /* ========================================
         * Compute Bounding Boxes
         * ========================================*/
        std::vector<geometry_msgs::msg::Point> line_list = minMax2lines(max_min_pts);
        std::vector<geometry_msgs::msg::Point> line_list2 = minMax2lines(max_min_pts2);
        

        /* ========================================
         * CONVERT PointCloud2 PCL->ROS, PUBLISH CLOUD
         * ========================================*/
        // this->publishPointCloud(voxel_grid_pub_, *cloud_voxel_filtered);
        this->publishPointCloud(plane_pub_, *cloud_f);
        this->publishPointCloud(euclidean_pub_, *clusters[0]);
        this->publishPointCloud(stat_pub_, sor_cloud_filtered);

        this->publish3DBBox(marker_pub_, line_list);
        // this->publish3DBBoxOBB(marker_pub_, line_list2);

        auto stop = std::chrono::high_resolution_clock::now();
        auto t_ms = std::chrono::duration_cast<std::chrono::milliseconds>(stop - start);
        RCLCPP_INFO(get_logger(), "Time (msec): %ld", t_ms.count());
    }

    void publishPointCloud(rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher,
                           pcl::PointCloud<pcl::PointXYZI> point_cloud)
    {
        sensor_msgs::msg::PointCloud2::SharedPtr pc2_cloud(new sensor_msgs::msg::PointCloud2);
        pcl::toROSMsg(point_cloud, *pc2_cloud);
        pc2_cloud->header.frame_id = world_frame;
        pc2_cloud->header.stamp = this->get_clock()->now();
        publisher->publish(*pc2_cloud);
    }

    void publish3DBBox(rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr publisher,
        const std::vector<geometry_msgs::msg::Point>& line_list)
    {
        visualization_msgs::msg::Marker::SharedPtr bboxes(new visualization_msgs::msg::Marker);
        bboxes->header.frame_id = world_frame;
        bboxes->header.stamp = this->get_clock()->now();
        bboxes->type = visualization_msgs::msg::Marker::LINE_LIST;
        bboxes->action = visualization_msgs::msg::Marker::ADD;
        bboxes->points = line_list;
        bboxes->scale.x = 0.06;
        bboxes->scale.y = 0.1;
        bboxes->scale.z = 0.1;
        bboxes->color.g = 1.0;
        bboxes->color.a = 1.0;
        // bboxes->pose.orientation.w = 1.0;

        publisher->publish(*bboxes);
    }

    void publish3DBBoxOBB(rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr publisher,
        const std::vector<geometry_msgs::msg::Point>& line_list)
    {
        visualization_msgs::msg::Marker::SharedPtr bboxes(new visualization_msgs::msg::Marker);
        bboxes->header.frame_id = world_frame;
        bboxes->header.stamp = this->get_clock()->now();
        bboxes->type = visualization_msgs::msg::Marker::LINE_LIST;
        bboxes->action = visualization_msgs::msg::Marker::ADD;
        bboxes->points = line_list;
        bboxes->scale.x = 0.06;
        bboxes->scale.y = 0.1;
        bboxes->scale.z = 0.1;

        // Set colour
        bboxes->color.r = 0.2;
        bboxes->color.b = 0.3;
        bboxes->color.g = 1.0;
        bboxes->color.a = 1.0;

        // Set pose
        // bboxes->pose.position.x = translation.x();
        // bboxes->pose.position.y = translation.y();
        // bboxes->pose.position.z = translation.z();
        // bboxes->pose.orientation.x = quat.x();
        // bboxes->pose.orientation.y = quat.y();
        // bboxes->pose.orientation.z = quat.z();
        // bboxes->pose.orientation.w = quat.w();

        publisher->publish(*bboxes);
    }

    std::vector<geometry_msgs::msg::Point> minMax2lines(CubePoints &max_min_pts)
    {
        std::vector<geometry_msgs::msg::Point> out;
        geometry_msgs::msg::Point p0, p1, v1, v2, v3, v4, v5, v6, v7, v8;

        auto i = max_min_pts.max_pts.begin();
        auto j = max_min_pts.min_pts.begin();
        while (i != max_min_pts.max_pts.end() and j != max_min_pts.min_pts.end())
        {
            auto p1 = *i++;
            auto p2 = *j++;

            // Create 8 vertices of a cube
            v1.x = p1[X];
            v1.y = p1[Y];
            v1.z = p1[Z];

            v2.x = p2[X];
            v2.y = p1[Y];
            v2.z = p1[Z];

            v3.x = p2[X];
            v3.y = p2[Y];
            v3.z = p1[Z];

            v4.x = p1[X];
            v4.y = p2[Y];
            v4.z = p1[Z];

            v5.x = p1[X];
            v5.y = p1[Y];
            v5.z = p2[Z];

            v6.x = p2[X];
            v6.y = p1[Y];
            v6.z = p2[Z];

            v7.x = p2[X];
            v7.y = p2[Y];
            v7.z = p2[Z];

            v8.x = p1[X];
            v8.y = p2[Y];
            v8.z = p2[Z];

            // Append points in pairs for marker LINE_LIST to create cube edges
            out.insert(out.end(), {v1,v2, v2,v3, v1,v4, v3,v4,
                                    v1,v5, v3,v7, v4,v8, v2,v6,
                                    v5,v8, v7,v8, v5,v6, v6,v7});
        }
        return out;
    }

    std::vector<geometry_msgs::msg::Point> minMax2lines(CubePointsPCL &max_min_pts)
    {
        std::vector<geometry_msgs::msg::Point> out;
        geometry_msgs::msg::Point p0, p1, v1, v2, v3, v4, v5, v6, v7, v8;

        auto i = max_min_pts.max_pts.begin();
        auto j = max_min_pts.min_pts.begin();
        while (i != max_min_pts.max_pts.end() and j != max_min_pts.min_pts.end())
        {
            auto p1 = *i++;
            auto p2 = *j++;

            // Create 8 vertices of a cube
            v1.x = p1[X];
            v1.y = p1[Y];
            v1.z = p1[Z];

            v2.x = p2[X];
            v2.y = p1[Y];
            v2.z = p1[Z];

            v3.x = p2[X];
            v3.y = p2[Y];
            v3.z = p1[Z];

            v4.x = p1[X];
            v4.y = p2[Y];
            v4.z = p1[Z];

            v5.x = p1[X];
            v5.y = p1[Y];
            v5.z = p2[Z];

            v6.x = p2[X];
            v6.y = p1[Y];
            v6.z = p2[Z];

            v7.x = p2[X];
            v7.y = p2[Y];
            v7.z = p2[Z];

            v8.x = p1[X];
            v8.y = p2[Y];
            v8.z = p2[Z];

            // Append points in pairs for marker LINE_LIST to create cube edges
            out.insert(out.end(), {v1,v2, v2,v3, v1,v4, v3,v4,
                                    v1,v5, v3,v7, v4,v8, v2,v6,
                                    v5,v8, v7,v8, v5,v6, v6,v7});
        }
        return out;
    }

    /*
     * Sub and Pub
     */
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_subscriber_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr voxel_grid_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr passthrough_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr plane_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr euclidean_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr stat_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr polygon_pub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;

    /*
     * Parameters
     */
    std::string cloud_topic;
    std::string world_frame;
    std::string camera_frame;
    float voxel_leaf_size;
    float x_filter_min;
    float x_filter_max;
    float y_filter_min;
    float y_filter_max;
    float z_filter_min;
    float z_filter_max;
    int plane_max_iter;
    float plane_dist_thresh;
    float cluster_tol;
    int cluster_min_size;
    int cluster_max_size;
    size_t median_filter_len;

    /*
     * TF
     */
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> br;
};


int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PerceptionNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
