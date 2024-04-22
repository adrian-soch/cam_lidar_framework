/**
 * @file transform_explorer_node.cpp
 * @author Adrian Sochaniwsky
 */

#include "lidar_pipeline/point_cloud_utils.hpp"

#include "rclcpp/rclcpp.hpp"
#include <builtin_interfaces/msg/time.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/transforms.hpp>

class TransformExplorer : public rclcpp::Node
{
public:
    TransformExplorer()
        : Node("transform_explorer", rclcpp::NodeOptions())
    {
        RCLCPP_INFO(this->get_logger(), "Initializing node...");

        // Declare parameters and default values
        this->declare_parameter("cloud_topic", "/points");
        this->declare_parameter("world_frame", "map");
        this->declare_parameter("voxel_leaf_size", 0.16);

        this->get_parameter("cloud_topic", cloud_topic);
        this->get_parameter("world_frame", world_frame);

        // Declare a map of parameters with names and values
        std::map<std::string, rclcpp::ParameterValue> t_params;
        t_params["x"] = rclcpp::ParameterValue(0.0);
        t_params["y"] = rclcpp::ParameterValue(0.0);
        t_params["z"] = rclcpp::ParameterValue(0.0);
        this->declare_parameters("t.position", t_params);

        // Declare a map of parameters with names and values
        std::map<std::string, rclcpp::ParameterValue> o_params;
        o_params["roll"]  = rclcpp::ParameterValue(0.0);
        o_params["pitch"] = rclcpp::ParameterValue(0.0);
        o_params["yaw"]   = rclcpp::ParameterValue(0.0);
        this->declare_parameters("t.orientation", o_params);

        crop_pub_         = this->create_publisher<sensor_msgs::msg::PointCloud2>("clouds/crop_cluster", 1);
        stat_pub_         = this->create_publisher<sensor_msgs::msg::PointCloud2>("clouds/stat_cluster", 1);
        cloud_subscriber_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            cloud_topic, 1, std::bind(&TransformExplorer::cloud_callback, this, std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(), "Init complete.");
    }

private:

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_subscriber_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr crop_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr stat_pub_;

    /*
     * Parameters
     */
    std::string cloud_topic;
    std::string world_frame;
    float voxel_leaf_size;
    std::vector<double> lidar2world_translation;
    std::vector<double> lidar2world_quat;

    builtin_interfaces::msg::Time stamp_;

    // Create cloud operation object
    Operations<pcl::PointXYZI> cloud_ops;

    void cloud_callback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr recent_cloud)
    {
        stamp_ = recent_cloud->header.stamp;
        auto start = std::chrono::high_resolution_clock::now();

        // Get values
        this->get_parameter("voxel_leaf_size", voxel_leaf_size);

        std::map<std::string, rclcpp::ParameterValue> t_params;
        this->get_parameters("t.position", t_params);

        std::map<std::string, rclcpp::ParameterValue> o_params;
        std::vector<rclcpp::Parameter> o_values;
        this->get_parameters("t.orientation", o_params);

        /* ========================================
         * CONVERT PointCloud2 ROS->PCL
         * ========================================*/
        pcl::PointCloud<pcl::PointXYZI> cloud;
        pcl::fromROSMsg(*recent_cloud, cloud);

        Eigen::Affine3f transform = params2transform(t_params, o_params);
        pcl::transformPointCloud(cloud, cloud, transform);

        /* ========================================
         * VOXEL GRID
         * ========================================*/
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_ptr(new pcl::PointCloud<pcl::PointXYZI>(cloud));
        cloud_ops.voxel_grid_filter(cloud_ptr, voxel_leaf_size, voxel_leaf_size, voxel_leaf_size);

        /* ========================================
         * CROPBOX
         * ========================================*/
        pcl::PointCloud<pcl::PointXYZI>::Ptr crop_cloud_ptr(new pcl::PointCloud<pcl::PointXYZI>(*cloud_ptr));

        /* ========================================
         * STATISTICAL OUTLIER REMOVAL
         * ========================================*/
        pcl::PointCloud<pcl::PointXYZI>::Ptr stats_cloud_ptr(new pcl::PointCloud<pcl::PointXYZI>(*crop_cloud_ptr));
        cloud_ops.stats_outlier_removal(stats_cloud_ptr, 50, 10.0);

        /* ========================================
         * CONVERT PointCloud2 PCL->ROS, PUBLISH CLOUD
         * ========================================*/
        this->publishPointCloud(crop_pub_, *crop_cloud_ptr);
        this->publishPointCloud(stat_pub_, *stats_cloud_ptr);

        auto stop = std::chrono::high_resolution_clock::now();
        auto t_ms = std::chrono::duration_cast<std::chrono::milliseconds>(stop - start);
        RCLCPP_INFO(get_logger(), "Time (msec): %ld", t_ms.count());
    } // cloud_callback

    template<typename PointT>
    void publishPointCloud(rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher,
      const pcl::PointCloud<PointT>                                                    &point_cloud)
    {
        sensor_msgs::msg::PointCloud2::SharedPtr pc2_cloud(new sensor_msgs::msg::PointCloud2);

        pcl::toROSMsg(point_cloud, *pc2_cloud);
        pc2_cloud->header.frame_id = world_frame;
        pc2_cloud->header.stamp    = stamp_;
        publisher->publish(*pc2_cloud);
    }

    Eigen::Affine3f params2transform(std::map<std::string, rclcpp::ParameterValue> t_params,
      std::map<std::string, rclcpp::ParameterValue> o_params)
    {
        tf2::Quaternion quat;

        quat.setRPY(o_params.at("roll").get<double>(), o_params.at("pitch").get<double>(), o_params.at(
              "yaw").get<double>());
        quat = quat.normalize();

        Eigen::Affine3f transform = Eigen::Affine3f::Identity();
        transform.translation() << t_params.at("x").get<double>(), t_params.at("y").get<double>(),
            t_params.at("z").get<double>();
        transform.rotate(Eigen::Quaternionf(quat.getW(), quat.getX(),
          quat.getY(), quat.getZ()));

        RCLCPP_INFO(get_logger(), "Orientation (W, X, Y, Z): [%f, %f, %f, %f]", quat.getW(), quat.getX(),
          quat.getY(), quat.getZ());
        RCLCPP_INFO(get_logger(), "Translation (X, Y, Z): [%f, %f, %f]\n", t_params.at("x").get<double>(), t_params.at(
              "y").get<double>(),
          t_params.at("z").get<double>());

        return transform;
    }
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TransformExplorer>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
