/**
 * @file perception_node.hpp
 * @brief Process LiDAR pointcloud data
 * @author Adrian Sochaniwsky (sochania@mcmaster.ca)
 */

#ifndef LIDAR_PROCESSING_HPP_
#define LIDAR_PROCESSING_HPP_

// ROS Message Includes
#include <geometry_msgs/msg/point.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <vision_msgs/msg/bounding_box3_d.hpp>
#include <vision_msgs/msg/detection3_d.hpp>
#include <vision_msgs/msg/detection3_d_array.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

// Application specific headers
#include "lidar_pipeline/pcl_dbscan.hpp"
#include "lidar_pipeline/point_cloud_utils.hpp"
#include "pipeline_interfaces/msg/point_cloud2_array.hpp"

namespace lidar_pipeline
{
class LidarProcessing : public rclcpp::Node
{
public:
    LidarProcessing()
        : Node("perception_node", rclcpp::NodeOptions())
    {
        /*
         * SET UP PUBLISHERS
         */
        RCLCPP_INFO(this->get_logger(), "Setting up publishers");

        voxel_grid_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("clouds/voxel_cluster", 1);
        crop_pub_       = this->create_publisher<sensor_msgs::msg::PointCloud2>("clouds/crop_cluster", 1);
        plane_pub_      = this->create_publisher<sensor_msgs::msg::PointCloud2>("clouds/plane_cluster", 1);
        stat_pub_       = this->create_publisher<sensor_msgs::msg::PointCloud2>("clouds/stat_cluster", 1);

        marker_pub_             = this->create_publisher<visualization_msgs::msg::Marker>("lidar_proc/aabboxes", 1);
        marker_array_pub_       = this->create_publisher<visualization_msgs::msg::MarkerArray>("lidar_proc/obboxes", 1);
        range_marker_array_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("lidar_proc/ranges", 1);

        aa_detection_pub_ = this->create_publisher<vision_msgs::msg::Detection3DArray>("lidar_proc/aa_detections", 1);
        o_detection_pub_  = this->create_publisher<vision_msgs::msg::Detection3DArray>("lidar_proc/o_detections", 1);

        pc_array_pub_ = this->create_publisher<pipeline_interfaces::msg::PointCloud2Array>("lidar_proc/obj_clouds", 1);

        /*
         * SET UP PARAMETERS (COULD BE INPUT FROM LAUNCH FILE/TERMINAL)
         */

        RCLCPP_INFO(this->get_logger(), "Getting parameters");

        // Declare parameters and default values
        this->declare_parameter("cloud_topic", "/points");
        this->declare_parameter("world_frame", "map");
        this->declare_parameter("camera_frame", "laser_data_frame");
        this->declare_parameter("voxel_leaf_size_x", 0.1);
        this->declare_parameter("voxel_leaf_size_y", 0.1);
        this->declare_parameter("voxel_leaf_size_z", 0.12);
        this->declare_parameter("plane_max_iter", 120);
        this->declare_parameter("plane_dist_thresh", 0.35);
        this->declare_parameter("cluster_tol", 1.35);
        this->declare_parameter("cluster_min_size", 3);
        this->declare_parameter("cluster_max_size", 10000);
        this->declare_parameter<std::vector<double> >("lidar2world_transform.translation", { 0.0 });
        this->declare_parameter<std::vector<double> >("lidar2world_transform.quaternion", { 1.0, 0.0, 0.0, 0.0 });
        this->declare_parameter<std::vector<double> >("crop_box_transform.translation", { -1. });
        this->declare_parameter<std::vector<double> >("crop_box_transform.quaternion", { -1., -1., -1., -1. });
        this->declare_parameter<std::vector<double> >("crop_box_transform.size", { -1., -1., -1. });

        // Get values from cmd line or YAML
        this->get_parameter("cloud_topic", cloud_topic);
        this->get_parameter("world_frame", world_frame);
        this->get_parameter("camera_frame", camera_frame);
        this->get_parameter("voxel_leaf_size_x", voxel_leaf_size_x);
        this->get_parameter("voxel_leaf_size_y", voxel_leaf_size_y);
        this->get_parameter("voxel_leaf_size_z", voxel_leaf_size_z);
        this->get_parameter("plane_max_iter", plane_max_iter);
        this->get_parameter("plane_dist_thresh", plane_dist_thresh);
        this->get_parameter("cluster_tol", cluster_tol);
        this->get_parameter("cluster_min_size", cluster_min_size);
        this->get_parameter("cluster_max_size", cluster_max_size);
        this->get_parameter("lidar2world_transform.translation", lidar2world_translation);
        this->get_parameter("lidar2world_transform.quaternion", lidar2world_quat);
        this->get_parameter("crop_box_transform.translation", crop_box_translation[0]);
        this->get_parameter("crop_box_transform.quaternion", crop_box_quat[0]);
        this->get_parameter("crop_box_transform.size", crop_box_size[0]);

        /*
         * SET UP SUBSCRIBER
         */
        RCLCPP_INFO(this->get_logger(), "Setting up subscriber");

        cloud_subscriber_ =
          this->create_subscription<sensor_msgs::msg::PointCloud2>(
            cloud_topic, 1, std::bind(&LidarProcessing::cloud_callback, this, std::placeholders::_1));

        dbscan.set_params(1.0, 3);
    }

private:
    struct CubePoints {
        std::vector<Eigen::Vector4f> max_pts;
        std::vector<Eigen::Vector4f> min_pts;
    };
    enum Axis { X, Y, Z };

    /*
     * Sub and Pub
     */
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_subscriber_;

    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr voxel_grid_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr crop_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr plane_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr stat_pub_;

    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_array_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr range_marker_array_pub_;

    rclcpp::Publisher<vision_msgs::msg::Detection3DArray>::SharedPtr aa_detection_pub_;
    rclcpp::Publisher<vision_msgs::msg::Detection3DArray>::SharedPtr o_detection_pub_;


    rclcpp::Publisher<pipeline_interfaces::msg::PointCloud2Array>::SharedPtr pc_array_pub_;

    /*
     * Parameters
     */
    std::string cloud_topic;
    std::string world_frame;
    std::string camera_frame;
    float voxel_leaf_size_x, voxel_leaf_size_y, voxel_leaf_size_z;
    int plane_max_iter;
    float plane_dist_thresh;
    float cluster_tol;
    int cluster_min_size;
    int cluster_max_size;
    std::vector<double> lidar2world_translation;
    std::vector<double> lidar2world_quat;
    std::vector<std::vector<double> > crop_box_translation{ { }, { } };
    std::vector<std::vector<double> > crop_box_quat{ { }, { } };
    std::vector<std::vector<double> > crop_box_size{ { }, { } };

    // To keep track of frames processed
    int frame_count_ { 0 };

    // For assigning the same stamp in message headers
    builtin_interfaces::msg::Time stamp_;

    // Create cloud operation object
    Operations<pcl::PointXYZI> cloud_ops;

    DBSCAN<pcl::PointXYZI> dbscan;

    /**
     * @brief Executed when a point cloud is received. Must execute faster
     *          than the pointcloud publishing period. (Typically 100ms)
     *
     * @param recent_cloud
     */
    void
    cloud_callback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr recent_cloud);

    /**
     * @brief Get axis aligned bounding box object
     *
     * @tparam PointT
     * @param cloud_cluster
     * @return vision_msgs::msg::BoundingBox3D
     */
    template<typename PointT>
    vision_msgs::msg::BoundingBox3D
    getAxisAlignedBoudingBox(const pcl::PointCloud<PointT> &cloud_cluster);

    /**
     * @brief Get the Oriented Bouding Box object
     *
     * @tparam PointT
     * @param cloud_cluster
     * @return vision_msgs::msg::BoundingBox3D
     */
    template<typename PointT>
    vision_msgs::msg::BoundingBox3D
    getOrientedBoudingBox(const pcl::PointCloud<PointT> &cloud_cluster);

    /**
     * @brief Calls classifier service and returns ID
     *
     * @param bb
     * @param cloud_cluster
     * @return std::string
     */
    std::string
    classify(const vision_msgs::msg::BoundingBox3D bb, pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_cluster);

    /**
     * @brief Get the Bbox Color R G B A object
     *
     * @param id
     * @param out
     */
    void
    getBboxColorRGBA(const std::string id, std_msgs::msg::ColorRGBA* out);

    /**
     * @brief Publish a pointcloud
     *
     * @param publisher
     * @param point_cloud
     */
    template<typename PointT>
    void
    publishPointCloud(rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher,
      const pcl::PointCloud<PointT>                                               &point_cloud);

    void
    publish3DBBox(rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr publisher,
      const std::vector<geometry_msgs::msg::Point>                              & line_list);

    /**
     * @brief Publish the 3D boubnding box as a cube list marker array
     *
     * @param publisher
     * @param bboxes
     */
    void
    publish3DBBoxOBB(rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr publisher,
      const std::vector<vision_msgs::msg::Detection3D>                                  & bboxes);

    /**
     * @brief Publish 3D object detections
     *
     * @param publisher
     * @param detections
     */
    void
    publishDetections(rclcpp::Publisher<vision_msgs::msg::Detection3DArray>::SharedPtr publisher,
      const std::vector<vision_msgs::msg::Detection3D>                                 & detections);

    /**
     * @brief Convert 3D min/max points into a set of points representing vertices of a cube
     *          the vertices are in pairs that create each line segment of a 3D box
     *
     * @param max_min_pts
     * @return std::vector<geometry_msgs::msg::Point>
     */
    std::vector<geometry_msgs::msg::Point>
    minMax2lines(CubePoints &max_min_pts);

    /**
     * @brief Publish range markers with respect to the sensor position
     *
     * @param publisher
     */
    void
    publishDistanceMarkers(rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr publisher);

    /**
     * @brief Publish a vector of pointclouds as a custom pointcloudarray message
     *
     * @param clusters  std::vector of point cloud pointers
     * @param publisher
     */
    void
    publishPointCloudArray(rclcpp::Publisher<pipeline_interfaces::msg::PointCloud2Array>::SharedPtr publisher,
      std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr>                                             & clusters);
};
} // end namespace perceptions_node
#endif // ifndef LIDAR_PROCESSING_HPP_
