#include <cv_bridge/cv_bridge.h>
#include <dirent.h>
#include <opencv2/highgui/highgui.hpp>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/header.hpp>

class ImagePointCloudPublisher : public rclcpp::Node
{
    using PType = pcl::PointXYZ;
public:
    ImagePointCloudPublisher() : Node("image_pointcloud_publisher")
    {
        image_pub_      = this->create_publisher<sensor_msgs::msg::Image>("image", 10);
        pointcloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("points", 10);

        // Get the folder paths for image and point cloud files
        this->declare_parameter<std::string>("image_folder", "");
        this->declare_parameter<std::string>("pointcloud_folder", "");
        this->get_parameter("image_folder", image_folder_);
        this->get_parameter("pointcloud_folder", pointcloud_folder_);

        this->declare_parameter<bool>("use_system_time", false);
        this->get_parameter("use_system_time", use_system_time_);

        // Get the frame id for image and point cloud messages
        this->declare_parameter<std::string>("frame_id", "map");
        this->get_parameter("frame_id", frame_id_);

        // Get the publish rate
        this->declare_parameter<double>("publish_rate", 2.5);
        double publish_rate;
        this->get_parameter("publish_rate", publish_rate);

        image_files_      = get_files_in_folder(image_folder_);
        pointcloud_files_ = get_files_in_folder(pointcloud_folder_);

        if((image_files_.empty() || pointcloud_files_.empty()) ||
          image_files_.size() != pointcloud_files_.size())
        {
            RCLCPP_ERROR(this->get_logger(), "No files found in the folders");
            return;
        }

        timer_ = this->create_wall_timer(
            std::chrono::duration<double>(1.0 / publish_rate),
            std::bind(&ImagePointCloudPublisher::timer_callback, this));
    }

private:

    void timer_callback()
    {
        auto start = std::chrono::high_resolution_clock::now();

        RCLCPP_INFO(get_logger(), "Remaining files: %ld", image_files_.size() - file_index_);

        if(file_index_ >= image_files_.size() || file_index_ >= pointcloud_files_.size()) {
            exit(EXIT_SUCCESS);
        }

        // Load the image file using OpenCV and convert it to a ROS message
        cv::Mat image = cv::imread(image_files_[file_index_], cv::IMREAD_COLOR);
        if(image.empty()) {
            RCLCPP_ERROR(this->get_logger(), "Failed to load image file %s", image_files_[file_index_].c_str());
            return;
        }
        sensor_msgs::msg::Image::SharedPtr image_msg =
          cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", image).toImageMsg();

        // Load the point cloud file using PCL and convert it to a ROS message
        pcl::PointCloud<PType>::Ptr cloud(new pcl::PointCloud<PType> );
        if(pcl::io::loadPCDFile<PType>(pointcloud_files_[file_index_], *cloud) == -1) {
            RCLCPP_ERROR(this->get_logger(), "Failed to load point cloud file %s",
              pointcloud_files_[file_index_].c_str());
            return;
        }
        sensor_msgs::msg::PointCloud2::SharedPtr pointcloud_msg(new sensor_msgs::msg::PointCloud2);
        pcl::toROSMsg(*cloud, *pointcloud_msg);

        // Set the frame id and timestamp for both messages
        image_msg->header.frame_id      = frame_id_;
        pointcloud_msg->header.frame_id = frame_id_;

        if(true == use_system_time_) {
            image_msg->header.stamp      = this->now();
            pointcloud_msg->header.stamp = this->now();
        } else {
            std::vector<int> image_times = get_ros_time_from_name(image_files_[file_index_]);
            std::vector<int> cloud_times = get_ros_time_from_name(pointcloud_files_[file_index_]);

            image_msg->header.stamp.sec     = image_times[0];
            image_msg->header.stamp.nanosec = image_times[1];

            pointcloud_msg->header.stamp.sec     = cloud_times[0];
            pointcloud_msg->header.stamp.nanosec = cloud_times[1];
        }

        // Publish the image and point cloud messages
        image_pub_->publish(*image_msg);
        pointcloud_pub_->publish(*pointcloud_msg);

        RCLCPP_INFO(this->get_logger(), "Published image and point cloud from files %s and %s",
          image_files_[file_index_].c_str(), pointcloud_files_[file_index_].c_str());

        auto stop = std::chrono::high_resolution_clock::now();
        auto t_ms = std::chrono::duration_cast<std::chrono::milliseconds>(stop - start);
        RCLCPP_INFO(get_logger(), "Time to publish cloud and image (msec): %ld", t_ms.count());

        file_index_++;
    } // timer_callback

    std::vector<std::string> get_files_in_folder(const std::string& folder)
    {
        std::vector<std::string> files;
        DIR* dir = opendir(folder.c_str());

        if(dir == NULL) {
            RCLCPP_ERROR(this->get_logger(), "Failed to open folder %s", folder.c_str());
            return files;
        }

        // Read the files in the folder
        struct dirent* entry;
        while((entry = readdir(dir)) != NULL) {
            // Skip the current and parent directories
            if(strcmp(entry->d_name, ".") == 0 || strcmp(entry->d_name, "..") == 0) {
                continue;
            }
            // Append the file name to the folder path
            std::string file = folder + "/" + entry->d_name;
            files.push_back(file);
        }

        // Close the folder directory
        closedir(dir);

        // Sort the files alphabetically
        std::sort(files.begin(), files.end());

        return files;
    }

    std::vector<int> get_ros_time_from_name(std::string file_path)
    {
        size_t pos       = file_path.find_last_of("/");
        std::string name = file_path.substr(pos + 1);

        std::stringstream ss(name);
        int sec, nsec;
        std::string temp;

        std::getline(ss, temp, '_');
        std::getline(ss, temp, '_');
        sec = std::stoi(temp);
        std::getline(ss, temp, '.');
        nsec = std::stoi(temp);

        return std::vector<int> { sec, nsec };
    }

    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::string image_folder_, pointcloud_folder_;
    std::vector<std::string> image_files_, pointcloud_files_;
    std::string frame_id_;
    size_t file_index_ { 0 };
    bool use_system_time_ { false };
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ImagePointCloudPublisher>());
    rclcpp::shutdown();
    return 0;
}
