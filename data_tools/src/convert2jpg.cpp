#include <chrono>
#include <filesystem>
#include <iomanip>
#include <memory>
#include <sstream>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"

#include "cv_bridge/cv_bridge.h"
#include <opencv2/opencv.hpp>


class ImageSaver : public rclcpp::Node
{
public:
    explicit ImageSaver(const std::string& topic_name, const std::string& file_path, const bool flip)
        : Node("image_saver"), file_path_(file_path)
    {
        // Create folder
        std::filesystem::path path = file_path_;

        std::filesystem::create_directories(path);

        flip_ = flip;

        subscription_ = create_subscription<sensor_msgs::msg::Image>(
            topic_name, 10, std::bind(&ImageSaver::imageCallback, this, std::placeholders::_1));
        RCLCPP_INFO(this->get_logger(), "Subscribed to topic '%s'", topic_name.c_str());
    }

private:
    void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        im_prt = cv_bridge::toCvCopy(msg, msg->encoding);

        // flip image because webcam is upsidedown
        if(true == flip_) {
            cv::flip(im_prt->image, im_prt->image, -1);
        }

        // Generate a file name for the image based on the current time
        std::stringstream ss;
        ss << file_path_ << std::setfill('0') << std::setw(6) << count_ << "_" << msg->header.stamp.sec << "_"
           << std::setfill('0') << std::setw(9) << msg->header.stamp.nanosec << ".jpg";
        std::string file_name = ss.str();

        count_ += 1;

        // Save the image to a file
        cv::imwrite(file_name, im_prt->image);

        RCLCPP_INFO(get_logger(), "Saved image to %s", file_name.c_str());
    }

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
    int count_ { 0 };
    std::string file_path_;
    bool flip_ { false };

    cv_bridge::CvImagePtr im_prt;
};

int main(int argc, char** argv)
{
    // Initialize the ROS 2 node
    rclcpp::init(argc, argv);
    if(!((argc == 3) || (argc == 4))) {
        RCLCPP_ERROR(rclcpp::get_logger(
              "image_saver"), "Usage: %s <topic_name:string> <file_path:string> <flip_image:any_int (Optional)>",
          argv[0]);
        return 1;
    }

    std::string topic_name = argv[1];
    std::string file_path  = argv[2];
    bool flip = false;
    if(argc == 4) {
        flip = true;
    }

    auto node = std::make_shared<ImageSaver>(topic_name, file_path, flip);

    // Spin the node until shut down
    rclcpp::spin(node);

    // Shut down ROS 2
    rclcpp::shutdown();
    return 0;
}
