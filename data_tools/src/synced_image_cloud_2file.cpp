#include <chrono>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "message_filters/subscriber.h"
#include "message_filters/sync_policies/approximate_time.h"

#include <opencv2/opencv.hpp>
#include "cv_bridge/cv_bridge.h"

#include "pcl/io/pcd_io.h"
#include "pcl/common/common.h"
#include "pcl_conversions/pcl_conversions.h"

#include <filesystem>

using namespace std::chrono_literals;

class SyncedSubscriberNode : public rclcpp::Node
{
public:
    SyncedSubscriberNode(const std::string& image_topic, const std::string& cloud_topic, const std::string& root_path)
       : Node("synced_subscriber_node"), pcd_path_(root_path)
    {
        // Set up image subscriber
        image_subscriber_ = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::Image>>(
        this, image_topic, rmw_qos_profile_sensor_data);

        // Set up point cloud subscriber
        cloud_subscriber_ = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::PointCloud2>>(
        this, cloud_topic, rmw_qos_profile_sensor_data);

        // Set up approximate time synchronizer
        approx_sync_ = std::make_shared<message_filters::Synchronizer<ApproximateSyncPolicy>>(
        ApproximateSyncPolicy(10), *image_subscriber_, *cloud_subscriber_);

        // Register the callback
        approx_sync_->registerCallback(std::bind(&SyncedSubscriberNode::imageCloudCallback, this, std::placeholders::_1, std::placeholders::_2));

        // Create folders
        img_path_ = pcd_path_ + "/related_images/";
        std::filesystem::path path = img_path_ ;
        std::filesystem::create_directories(path);
    }

private:
  // Define the approximate time synchronization policy
    typedef message_filters::sync_policies::ApproximateTime<
        sensor_msgs::msg::Image, sensor_msgs::msg::PointCloud2> ApproximateSyncPolicy;

  // Callback function for synchronized image and point cloud
    void imageCloudCallback(const sensor_msgs::msg::Image::ConstSharedPtr& image_msg,
                            const sensor_msgs::msg::PointCloud2::ConstSharedPtr& cloud_msg)
    {
        im_prt = cv_bridge::toCvCopy(image_msg, image_msg->encoding);

        // flip image because webcam is upsidedown
        cv::flip(im_prt->image, im_prt->image, -1);

        // Generate a file name for the image based on the current time
        std::stringstream ss;
        ss << img_path_ << std::setfill('0') << std::setw(6) << count_ << ".jpg";
        std::string file_name = ss.str();

        // Save the image to a file
        cv::imwrite(file_name, im_prt->image);

        RCLCPP_INFO(get_logger(), "Saved image to %s", file_name.c_str());



        // Convert and save to .pcd
        pcl::PointCloud<pcl::PointXYZI>::Ptr pcl_cloud(new pcl::PointCloud<pcl::PointXYZI>());
        pcl::fromROSMsg(*cloud_msg, *pcl_cloud);

        // Save PCD file with an increasing file name
        std::stringstream ss1;
        ss1 << pcd_path_ << std::setfill('0') << std::setw(6) << count_++ << ".pcd";
        std::string file_name1 = ss1.str();
        pcl::io::savePCDFileBinary(file_name1, *pcl_cloud);
        RCLCPP_INFO(this->get_logger(), "Saved PCD file to %s", file_name1.c_str());

        // Print the synchronized timestamp
        // auto timestamp = std::max(image_msg->header.stamp, cloud_msg->header.stamp);
        // RCLCPP_INFO(get_logger(), "Synchronized timestamp: %ld.%09ld", timestamp.sec, timestamp.nanosec);
    }

    int count_ {0};
    std::string img_path_, pcd_path_;
    cv_bridge::CvImagePtr im_prt;

    std::shared_ptr<message_filters::Subscriber<sensor_msgs::msg::Image>> image_subscriber_;
    std::shared_ptr<message_filters::Subscriber<sensor_msgs::msg::PointCloud2>> cloud_subscriber_;
    std::shared_ptr<message_filters::Synchronizer<ApproximateSyncPolicy>> approx_sync_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    if (argc != 4) {
        RCLCPP_ERROR(rclcpp::get_logger("synced_subscriber_node"), "Usage: %s <topic_name> <file_path>", argv[0]);
        return 1;
    }

    // Create a ROS 2 node and pass in the image and point cloud topic names as arguments
    auto node = std::make_shared<SyncedSubscriberNode>(argv[1], argv[2], argv[3]);
    rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;
}
