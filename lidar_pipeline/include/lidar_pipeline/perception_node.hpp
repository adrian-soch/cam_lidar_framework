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

#ifndef PERCEPTION_NODE_HPP_
#define PERCEPTION_NODE_HPP_

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

#include <pcl/common/common.h>
#include <pcl/common/io.h>

#include <tf2/convert.h>
#include <tf2_eigen/tf2_eigen.h>
#include <tf2/LinearMath/Quaternion.h>

namespace lidar_pipeline
{

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

    enum Axis {X, Y, Z};

    template <typename PointT>
    vision_msgs::msg::BoundingBox3D getOrientedBoudingBox(const pcl::PointCloud<PointT> &cloud_cluster)
    {
        Eigen::Vector4f centroid;
        pcl::compute3DCentroid(cloud_cluster, centroid);
        PointT min_pt, max_pt;
        Eigen::Vector3f center;
        pcl::getMinMax3D(cloud_cluster, min_pt, max_pt); 
        center = (max_pt.getVector3fMap() + min_pt.getVector3fMap())/2;
        
        Eigen::Matrix3f covariance;
        pcl::computeCovarianceMatrixNormalized(cloud_cluster, centroid, covariance);
        Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> eigen_solver(covariance, Eigen::ComputeEigenvectors);
        Eigen::Matrix3f eigenVectorsPCA = eigen_solver.eigenvectors();
        // Eigen::Vector3f eigenValuesPCA  = eigen_solver.eigenvalues();
        eigenVectorsPCA.col(2) = eigenVectorsPCA.col(0).cross(eigenVectorsPCA.col(1));
        eigenVectorsPCA.col(0) = eigenVectorsPCA.col(1).cross(eigenVectorsPCA.col(2));
        eigenVectorsPCA.col(1) = eigenVectorsPCA.col(2).cross(eigenVectorsPCA.col(0));

        Eigen::Matrix3f eigenVectorsPCA1;
        eigenVectorsPCA1.col(0) = eigenVectorsPCA.col(2);
        eigenVectorsPCA1.col(1) = eigenVectorsPCA.col(1);
        eigenVectorsPCA1.col(2) = eigenVectorsPCA.col(0);
        eigenVectorsPCA = eigenVectorsPCA1;

        Eigen::Vector3f ea = (eigenVectorsPCA).eulerAngles(2, 1, 0); //yaw pitch roll
        Eigen::AngleAxisf keep_Z_Rot(ea[0], Eigen::Vector3f::UnitZ());
        Eigen::Affine3f transform = Eigen::Affine3f::Identity();
        transform.translate(center);
        transform.rotate(keep_Z_Rot); // radians
        
        typename pcl::PointCloud<PointT>::Ptr transformedCloud(new pcl::PointCloud<PointT>);
        pcl::transformPointCloud(cloud_cluster, *transformedCloud, transform.inverse());

        PointT min_pt_T, max_pt_T;
        pcl::getMinMax3D(*transformedCloud, min_pt_T, max_pt_T);
        Eigen::Vector3f center_new = (max_pt_T.getVector3fMap() + min_pt_T.getVector3fMap()) / 2;
        Eigen::Vector3f box_dim;
        box_dim = max_pt_T.getVector3fMap() - min_pt_T.getVector3fMap();
        Eigen::Affine3f transform2 = Eigen::Affine3f::Identity();
        transform2.translate(center_new);
        // Eigen::Affine3f transform3 = transform * transform2;

        const Eigen::Quaternionf bboxQ(keep_Z_Rot);

        PointT size;
        size.getArray3fMap() = max_pt_T.getArray3fMap() - min_pt_T.getArray3fMap();

        vision_msgs::msg::BoundingBox3D bb;
        bb.center.position.x = centroid[X];
        bb.center.position.y = centroid[Y];
        bb.center.position.z = centroid[Z];
        bb.center.orientation.x = bboxQ.x();
        bb.center.orientation.y = bboxQ.y();
        bb.center.orientation.z = bboxQ.z();
        bb.center.orientation.w = bboxQ.w();
        bb.size.x = size.x;
        bb.size.y = size.y;
        bb.size.z = size.z;

        return bb;
    }

    void getBboxColorRGBA(const std::string id, std_msgs::msg::ColorRGBA *out)
    {
        // take in the id number
        // return colour via pointer based on id/object class



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

        publisher->publish(*bboxes);
    }

    void publish3DBBoxOBB(rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr publisher,
        const std::vector<vision_msgs::msg::Detection3D>& bboxes)
    {
        visualization_msgs::msg::MarkerArray marker_array;
        int idx = 0;
        for(auto c : bboxes) {
            visualization_msgs::msg::Marker marker;
            marker.id = idx++;
            marker.header.frame_id = world_frame;
            marker.header.stamp = this->get_clock()->now();
            marker.type = visualization_msgs::msg::Marker::CUBE;
            marker.action = visualization_msgs::msg::Marker::ADD;

            marker.pose = c.bbox.center; // Sets position and orientation
            marker.scale = c.bbox.size;  // Set w,l,h
            
            std_msgs::msg::ColorRGBA rgba;
            getBboxColorRGBA(c.id, &rgba); 
            marker.color = rgba;
            marker.color.a = 0.4;   // Set alpha so we can see underlying points

            marker_array.markers.push_back(marker);
        }
        publisher->publish(marker_array);
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

    /*
     * TF
     */
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> br;

    void cloud_callback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr recent_cloud);
};

} // end namespace perceptions_node

#endif