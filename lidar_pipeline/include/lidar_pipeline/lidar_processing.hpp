/**
 * @file perception_node.hpp
 * @brief Process LiDAR pointcloud data
 * @author Adrian Sochaniwsky (sochania@mcmaster.ca)
 * @version 0.1
 * @date 2023-03-05
 * 
 * @copyright Copyright (c) 2023
 * 
 */

#ifndef LIDAR_PROCESSING_HPP_
#define LIDAR_PROCESSING_HPP_

#include "lidar_pipeline/point_cloud_utils.hpp"

#include <iostream>

#include "rclcpp/rclcpp.hpp"
#include <rclcpp/qos.hpp>
#include <std_msgs/msg/color_rgba.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <vision_msgs/msg/bounding_box3_d.hpp>
#include <vision_msgs/msg/detection3_d.hpp>
#include <vision_msgs/msg/detection3_d_array.hpp>
#include <tf2/transform_datatypes.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>

#include <pcl_ros/transforms.hpp>
#include <pcl_conversions/pcl_conversions.h>

// #include <pcl/common/io.h>

#include <tf2/convert.h>
#include <tf2_eigen/tf2_eigen.h>
#include <tf2/LinearMath/Quaternion.h>

namespace lidar_pipeline
{

class LidarProcessing : public rclcpp::Node
{
public:
    LidarProcessing()
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
        marker_array_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("markers2", 1);

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
        this->get_parameter_or("z_filter_min", z_filter_min_param, rclcpp::Parameter("", -1.0));
        this->get_parameter_or("z_filter_max", z_filter_max_param, rclcpp::Parameter("", 8.0));
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
                cloud_topic, 1, std::bind(&LidarProcessing::cloud_callback, this, std::placeholders::_1));

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

    enum Axis {X, Y, Z};

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
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_array_pub_;
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

    // Create cloud operation object
    Operations<pcl::PointXYZI> cloud_ops;

    /*
     * TF
     */
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> br;


    /**
     * @brief 
     * 
     * @param recent_cloud 
     */
    void cloud_callback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr recent_cloud);

    /**
     * @brief Get the Oriented Bouding Box object
     * 
     * @tparam PointT 
     * @param cloud_cluster 
     * @return vision_msgs::msg::BoundingBox3D 
     */
    template <typename PointT>
    vision_msgs::msg::BoundingBox3D getOrientedBoudingBox(const pcl::PointCloud<PointT> &cloud_cluster);

    /**
     * @brief 
     * 
     * @param bb 
     * @return std::string 
     */
    std::string simpleClassifier(const vision_msgs::msg::BoundingBox3D bb);

    /**
     * @brief Get the Bbox Color R G B A object
     * 
     * @param id 
     * @param out 
     */
    void getBboxColorRGBA(const std::string id, std_msgs::msg::ColorRGBA *out);

    /**
     * @brief 
     * 
     * @param publisher 
     * @param point_cloud 
     */
    void publishPointCloud(rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher,
                           pcl::PointCloud<pcl::PointXYZI> point_cloud);

    void publish3DBBox(rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr publisher,
        const std::vector<geometry_msgs::msg::Point>& line_list);

    /**
     * @brief 
     * 
     * @param publisher 
     * @param bboxes 
     */
    void publish3DBBoxOBB(rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr publisher,
        const std::vector<vision_msgs::msg::Detection3D>& bboxes);

    /**
     * @brief 
     * 
     * @param max_min_pts 
     * @return std::vector<geometry_msgs::msg::Point> 
     */
    std::vector<geometry_msgs::msg::Point> minMax2lines(CubePoints &max_min_pts);
};

} // end namespace perceptions_node
#endif