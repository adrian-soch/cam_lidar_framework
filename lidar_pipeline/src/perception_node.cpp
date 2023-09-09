/**
 * @file perception_node.cpp
 * @brief Process LiDAR pointcloud data
 * @author Adrian Sochaniwsky (sochania@mcmaster.ca)
 * @version 0.1
 * @date 2023-01-12
 *
 * @copyright Copyright (c) 2023
 *
 */

#include "lidar_pipeline/lidar_processing.hpp"

#include "rclcpp/executors/multi_threaded_executor.hpp"

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<lidar_pipeline::LidarProcessing>();

    auto exec = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
    exec->add_node(node);
    exec->spin();


    rclcpp::shutdown();
    return 0;
}
