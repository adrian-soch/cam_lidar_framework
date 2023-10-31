/**
 * @file convert2pcd.cpp
 * @author your name (you@domain.com)
 * @brief
 * @version 0.1
 * @date 2023-04-22
 *
 *      For dec7_2022/roofTestDaylight_2_FHD_qosOverrride_true/:
 *          stransform.transform.rotation.y = 0.0998334;
 *          stransform.transform.rotation.w = 0.9950042;
 *          stransform.transform.translation.z = 12;
 *
 * @copyright Copyright (c) 2023
 *
 */
#include <chrono>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <memory>
#include <sstream>
#include <string>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include <pcl_ros/transforms.hpp>

#include "pcl/common/common.h"
#include "pcl/io/pcd_io.h"
#include "pcl_conversions/pcl_conversions.h"


class PointCloud2ToPCDConverter : public rclcpp::Node
{
public:
    explicit PointCloud2ToPCDConverter(const std::string& topic_name, const std::string& file_path)
        : Node("point_cloud2_to_pcd_converter"), file_path_(file_path)
    {
        // Create folder
        std::filesystem::path path = file_path_;

        std::filesystem::create_directories(path);

        subscription_ = create_subscription<sensor_msgs::msg::PointCloud2>(
            topic_name, 10, std::bind(&PointCloud2ToPCDConverter::callback, this, std::placeholders::_1));
        RCLCPP_INFO(this->get_logger(), "Subscribed to topic '%s'", topic_name.c_str());
    }

private:
    void callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) const
    {
        pcl::PointCloud<pcl::PointXYZI>::Ptr pcl_cloud(new pcl::PointCloud<pcl::PointXYZI>());

        pcl::fromROSMsg(*msg, *pcl_cloud);

        // Save PCD file with an increasing file name
        std::stringstream ss;
        ss << file_path_ << std::setfill('0') << std::setw(6) << file_count_++ << msg->header.stamp.sec << "_"
           << std::setfill('0') << std::setw(9) << msg->header.stamp.nanosec << ".pcd";
        std::string file_name = ss.str();

        pcl::io::savePCDFileBinary(file_name, *pcl_cloud);
        RCLCPP_INFO(this->get_logger(), "Saved PCD file to %s", file_name.c_str());
    }

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
    std::string file_path_;
    mutable int file_count_ = 0;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    if(argc != 3) {
        RCLCPP_ERROR(rclcpp::get_logger("PointCloud2ToPCDConverter"), "Usage: %s <topic_name> <file_path>", argv[0]);
        return 1;
    }

    std::string topic_name = argv[1];
    std::string file_path  = argv[2];

    auto node = std::make_shared<PointCloud2ToPCDConverter>(topic_name, file_path);
    rclcpp::spin(node);

    rclcpp::shutdown();
    return 0;
}
