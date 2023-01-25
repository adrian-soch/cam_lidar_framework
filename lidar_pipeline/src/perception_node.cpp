/**
 * @file perception_node.cpp
 * @brief Process LiDAR pointcloud data
 * @version 0.1
 * @date 2023-01-12
 *
 * @todo
 *      - Keep
 *      - Publish array of segmented clusters?
 *          - Add diff colour clusters into 1 cloud
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
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/segmentation/extract_polygonal_prism_data.h>

#include <tf2/convert.h>
#include <tf2_eigen/tf2_eigen.h>
#include <tf2/LinearMath/Quaternion.h>

template <typename PointT> void
myGetMinMax3D (const pcl::PointCloud<PointT> &cloud, Eigen::Vector4f &min_pt, Eigen::Vector4f &max_pt)
{
  Eigen::Array4f min_p, max_p;
  min_p.setConstant (FLT_MAX);
  max_p.setConstant (-FLT_MAX);

  // If the data is dense, we don't need to check for NaN
  if (cloud.is_dense)
  {
    for (const auto& point: cloud.points)
    {
      const auto pt = point.getArray4fMap ();
      min_p = min_p.min (pt);
      max_p = max_p.max (pt);
    }
  }
  // NaN or Inf values could exist => check for them
  else
  {
    for (const auto& point: cloud.points)
    {
      // Check if the point is invalid
      if (!std::isfinite (point.x) ||
          !std::isfinite (point.y) ||
          !std::isfinite (point.z))
        continue;
      const auto pt = point.getArray4fMap ();
      min_p = min_p.min (pt);
      max_p = max_p.max (pt);
    }
  }
  min_pt = min_p;
  max_pt = max_p;
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

        /*
         * SET UP PARAMETERS (COULD BE INPUT FROM LAUNCH FILE/TERMINAL)
         */
        rclcpp::Parameter cloud_topic_param, world_frame_param, camera_frame_param, voxel_leaf_size_param,
            x_filter_min_param, x_filter_max_param, y_filter_min_param, y_filter_max_param, z_filter_min_param,
            z_filter_max_param, plane_max_iter_param, plane_dist_thresh_param, cluster_tol_param,
            cluster_min_size_param, cluster_max_size_param;

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
    void cloud_callback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr recent_cloud)
    {
        RCLCPP_INFO(this->get_logger(), "Cloud service called; getting a PointCloud2 on topic");
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
        pcl::PointCloud<pcl::PointXYZ> cloud2;

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
        ;
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
        // std::vector<> lines;
        std::vector<Eigen::Vector4f> max_pts, min_pts;
        
        int j = 0;
        for (const auto &cluster : cluster_indices)
        {
            pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_cluster(new pcl::PointCloud<pcl::PointXYZI>);

            // Compute MinMax points ABB
            Eigen::Vector4f minPt{}, maxPt{};
            pcl::getMinMax3D(*cloud_filtered, cluster, minPt, maxPt);

            max_pts.push_back(maxPt);
            min_pts.push_back(minPt);

            // std::cout << "Min: " << minPt << std::endl;
            // std::cout << "Max: " << maxPt << std::endl;

            
            // Put each cluster into a vector
            for (const auto &idx : cluster.indices)
            {
                cloud_cluster->points.push_back((*cloud_filtered)[idx]);
            }
            cloud_cluster->width = cloud_cluster->points.size();
            cloud_cluster->height = 1;
            cloud_cluster->is_dense = true;
            // std::cout << cloud_cluster->points[1].intensity << std::endl;
            // RCLCPP_INFO(this->get_logger(), "Cluster has '%lu' points", cloud_cluster->points.size());
            clusters.push_back(cloud_cluster);

            j++;

            // Oriented Bounding Box (OBB)
            Eigen::Vector4f pcaCentroid;
            pcl::compute3DCentroid(*cloud_cluster, pcaCentroid);
        }
        std::sort(clusters.begin(), clusters.end());

        

        /* ========================================
         * BROADCAST TRANSFORM
         * ========================================*/
        // std::unique_ptr<tf2_ros::TransformBroadcaster> br = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
        // geometry_msgs::msg::TransformStamped part_transform;

        // tf2::Quaternion q;
        // q.setRPY(0, 0, 0);
        // part_transform.transform.rotation.x = q.x();
        // part_transform.transform.rotation.y = q.y();
        // part_transform.transform.rotation.z = q.z();
        // part_transform.transform.rotation.w = q.w();

        // //Here x,y, and z should be calculated based on the PointCloud2 filtering results
        // part_transform.transform.translation.x = sor_cloud_filtered->at(1).x;
        // part_transform.transform.translation.y = sor_cloud_filtered->at(1).y;
        // part_transform.transform.translation.z = sor_cloud_filtered->at(1).z;
        // part_transform.header.stamp = this->get_clock()->now();
        // part_transform.header.frame_id = world_frame;
        // part_transform.child_frame_id = "part";

        // br->sendTransform(part_transform);

        /* ========================================
         * CONVERT PointCloud2 PCL->ROS, PUBLISH CLOUD
         * ========================================*/
        this->publishPointCloud(voxel_grid_pub_, *cloud_voxel_filtered);
        this->publishPointCloud(plane_pub_, *cloud_f);
        this->publishPointCloud(euclidean_pub_, *clusters[0]);
        this->publishPointCloud(stat_pub_, *sor_cloud_filtered);

        this->publish3DBBox(marker_pub_, min_pts, max_pts);

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
         [[maybe_unused]] const std::vector<Eigen::Vector4f> &min,   [[maybe_unused]] const std::vector<Eigen::Vector4f> &max)
    {

        std::cout << "Here!\n";
        geometry_msgs::msg::Point p0, p1;
        p0.x = 0;
        p0.y = 0,
        p0.z = 0;

        p1.x = 10;
        p1.y = 10;
        p1.z = 10;
std::cout << "Here2\n";
        visualization_msgs::msg::Marker::SharedPtr bboxes(new visualization_msgs::msg::Marker);
        bboxes->header.frame_id = world_frame;
        bboxes->header.stamp = this->get_clock()->now();
        bboxes->id = bboxes->LINE_LIST;
        bboxes->type = bboxes->ADD;
        bboxes->points.push_back(p0); bboxes->points.push_back(p1);

        std::cout << "Here3\n";
        publisher->publish(*bboxes);
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
