#include <chrono>
#include <memory>
#include <string>
#include <sstream>
#include <iomanip>
#include <filesystem>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"

#include <opencv2/opencv.hpp>
#include "cv_bridge/cv_bridge.h"


class ImageSaver : public rclcpp::Node
{
public:
    explicit ImageSaver(const std::string& topic_name, const std::string& file_path)
        : Node("image_saver"), file_path_(file_path)
    {
        // Create folder
        std::filesystem::path path = file_path_;
        std::filesystem::create_directories(path);

        subscription_ = create_subscription<sensor_msgs::msg::Image>(
        topic_name, 10, std::bind(&ImageSaver::imageCallback, this, std::placeholders::_1));
        RCLCPP_INFO(this->get_logger(), "Subscribed to topic '%s'", topic_name.c_str());
    }

private:
    void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        im_prt = cv_bridge::toCvCopy(msg, msg->encoding);

        // Convert the Image message to an OpenCV Mat
        // cv::Mat image = cv::imdecode(cv::Mat(msg->height, msg->width, CV_8UC3, const_cast<uint8_t*>(msg->data.data())), cv::IMREAD_COLOR);

        // flip image because webcam is upsidedown
        cv::flip(im_prt->image, im_prt->image, -1);

        // Generate a file name for the image based on the current time
        std::stringstream ss;
        ss << file_path_ << std::setfill('0') << std::setw(6) << count_++ << ".jpg";
        std::string file_name = ss.str();

        // Save the image to a file
        cv::imwrite(file_name, im_prt->image);

        RCLCPP_INFO(get_logger(), "Saved image to %s", file_name.c_str());
    }

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
    int count_ {0};
    std::string file_path_;

    cv_bridge::CvImagePtr im_prt;
};

int main(int argc, char** argv)
{
    // Initialize the ROS 2 node
    rclcpp::init(argc, argv);
      if (argc != 3)
    {
        RCLCPP_ERROR(rclcpp::get_logger("image_saver"), "Usage: %s <topic_name> <file_path>", argv[0]);
        return 1;
    }

    std::string topic_name = argv[1];
    std::string file_path = argv[2];

    auto node = std::make_shared<ImageSaver>(topic_name, file_path);

    // Spin the node until shut down
    rclcpp::spin(node);

    // Shut down ROS 2
    rclcpp::shutdown();
    return 0;
}
