/**
 * @file main.cpp
 * @author Adrian Sochaniwsky
 * @brief Processes a 3D Pointcloud
 * @version 0.1
 * @date 2023-01-11
 * 
 * @copyright Copyright (c) 2023
 * 
 */

#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "pcl/filters/voxel_grid.h"

class LidarProc : public rclcpp::Node
{
  public:
    LidarProc()
    : Node("minimal_subscriber")
    {
      subscriber_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      "points", 5, std::bind(&LidarProc::topic_callback, this, std::placeholders::_1));

      publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("points2", 5);
    }

  private:
    void topic_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) const
    {
        // Create the filtering object
        // pcl::VoxelGrid<pcl::PCLPointCloud2> vox;
        // vox.setInputCloud (msg);
        // vox.setLeafSize (0.01f, 0.01f, 0.01f);
        // vox.filter (*cloud_filtered);
    }

    // void publish(const std_msgs::msg::Header::SharedPtr header, data)
    // {
    //     auto message = sensor_msgs::msg::PointCloud2();
    //     // message.data = "Hello, world!";
    //     RCLCPP_INFO(this->get_logger(), "Publishing");
    //     publisher_->publish(message);
    // }

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscriber_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LidarProc>());
  rclcpp::shutdown();
  return 0;
}