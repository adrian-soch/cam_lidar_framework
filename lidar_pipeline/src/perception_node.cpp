/**
 * @file perception_node.cpp
 * @brief Process LiDAR pointcloud data
 * @author Adrian Sochaniwsky (sochania@mcmaster.ca)
 *
 * @copyright Copyright (c) 2023
 */

#include <rclcpp/rclcpp.hpp>

#include "lidar_pipeline/lidar_processing.hpp"

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<lidar_pipeline::LidarProcessing>();
    auto exec = std::make_shared<rclcpp::executors::StaticSingleThreadedExecutor>();
    exec->add_node(node);
    exec->spin();

    rclcpp::shutdown();
    return 0;
}
