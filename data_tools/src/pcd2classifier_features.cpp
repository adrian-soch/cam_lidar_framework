/**
 * @file pcd2classifier_features.cpp
 * @author Adrian Sochaniwsky
 */

#include <ctime>
#include <dirent.h>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <jsoncpp/json/json.h>
#include <string>

#include <opencv2/highgui/highgui.hpp>

#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>


std::string generate_filename()
{
    // get the current time as a struct tm
    std::time_t t = std::time(nullptr);
    std::tm* tm   = std::localtime(&t);
    // create a string stream to format the time
    std::stringstream ss;

    // write the year, month, day, hour, minute, and second to the stream
    ss << tm->tm_year + 1900 << "-"
       << tm->tm_mon + 1 << "-"
       << tm->tm_mday << "_"
       << tm->tm_hour << "-"
       << tm->tm_min << "-"
       << tm->tm_sec;
    // return the string representation of the stream
    return ss.str();
}

class FeatureExtractor : public rclcpp::Node
{
public:
    FeatureExtractor() : Node("feature_extractor")
    {
        pointcloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("points", 10);

        std::string gt_folder;
        std::string pointcloud_folder;

        // Get the folder paths for ground_truths and point cloud files
        this->declare_parameter<std::string>("gt_folder", "");
        this->declare_parameter<std::string>("pointcloud_folder", "");
        this->declare_parameter<std::string>("path", "");
        this->get_parameter("gt_folder", gt_folder);
        this->get_parameter("pointcloud_folder", pointcloud_folder);
        this->get_parameter("path", path_);

        // Get the frame id for image and point cloud messages
        this->declare_parameter<std::string>("frame_id", "map");
        this->get_parameter("frame_id", frame_id_);

        gt_files_         = get_files_in_folder(gt_folder);
        pointcloud_files_ = get_files_in_folder(pointcloud_folder);

        // Check if there are any files to publish
        if((gt_files_.empty() || pointcloud_files_.empty()) ||
          gt_files_.size() != pointcloud_files_.size())
        {
            RCLCPP_ERROR(this->get_logger(), "No files found in the folders or file number mismatch.");
            return;
        }

        // Create the folder
        std::filesystem::path path = path_;
        std::filesystem::create_directories(path);

        cv::namedWindow("Keyboard Input", cv::WINDOW_NORMAL);

        RCLCPP_INFO(get_logger(), "Starting worker thread...");
        worker_thread();
    }

    ~FeatureExtractor ()
    {
        cv::destroyWindow("Keyboard Input");
    }

private:
    enum actions { STORE = 0, IGNORE, SHUTDOWN };
    enum geometry { X = 0, Y, Z, QX, QY, QZ, QW, W, L, H };

    typedef struct {
        std::string name;
        std::string type;
        int         num_points;
        double      length;
        double      width;
        double      height;
        double      dist2sensor;
        std::string occlusion_level;
    } Road_User_t;

    void worker_thread()
    {
        std::string file_path = path_ + "/" + generate_filename() + "_features.csv";

        while(true) {
            auto start = std::chrono::high_resolution_clock::now();

            // Increment the file index and exit if complete
            file_index_++;
            if(file_index_ >= gt_files_.size() || file_index_ >= pointcloud_files_.size()) {
                exit(EXIT_SUCCESS);
            }

            // Load the point cloud file using PCL and convert it to a ROS message
            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ> );
            if(pcl::io::loadPCDFile<pcl::PointXYZ>(pointcloud_files_[file_index_], *cloud) == -1) {
                RCLCPP_ERROR(this->get_logger(), "Failed to load point cloud file %s",
                  pointcloud_files_[file_index_].c_str());
                return;
            }

            // Read in json data
            Json::Value gt_data = read_json_file(gt_files_[file_index_]);

            // All label objects are inside the "frame" key
            Json::Value frame = gt_data["openlabel"]["frames"];
            std::vector<std::string> keys = frame.getMemberNames();

            // Get the frame number to access the objects
            std::string frame_num = keys[0];
            Json::Value objects   = frame[frame_num]["objects"];

            // Parse all "objects"
            std::vector<Road_User_t> road_users;
            for(Json::Value::const_iterator it = objects.begin(); it != objects.end(); ++it) {
                // RCLCPP_INFO(get_logger(), "%s", (*it).toStyledString().c_str());

                Road_User_t road_user;
                get_features(&road_user, it);

                // Print data
                RCLCPP_INFO(
                    get_logger(), "Name: %s \nType: %s \nNumPoints: %d \nL: %f\nW: %f \nH: %f \nDist2Sensor: %f \nOcc: %s\n",
                    road_user.name.c_str(),
                    road_user.type.c_str(), road_user.num_points, road_user.length, road_user.width,
                    road_user.height, road_user.dist2sensor, road_user.occlusion_level.c_str());


                // Segment the pointcloud to just include the object
                // @TODO

                // Publish segmented object
                // publish_pointcloud(cloud);

                // Wait for input from operator
                int action = user_input_handler();

                if(STORE == action) {
                    road_users.push_back(road_user);
                } else if(IGNORE == action) {
                    // do nothing
                } else if(SHUTDOWN == action) {
                    // Save what we have before shutdown
                    write_csv(road_users, file_path);
                    rclcpp::shutdown();
                    exit(EXIT_SUCCESS);
                }
            }

            // Savey save the csv file
            write_csv(road_users, file_path);

            auto stop = std::chrono::high_resolution_clock::now();
            auto t_ms = std::chrono::duration_cast<std::chrono::milliseconds>(stop - start);
            RCLCPP_INFO(get_logger(), "Time to publish cloud and image (msec): %ld", t_ms.count());
        }
    } // worker_thread

    void get_features(Road_User_t* road_user, Json::Value::const_iterator it)
    {
        std::vector<std::string> keys = (*it).getMemberNames();
        Json::Value data = (*it)["object_data"];

        double dist2sensor = sqrt(pow(data["cuboid"]["val"][X].asDouble(), 2.0)
            + pow(data["cuboid"]["val"][Y].asDouble(), 2.0)
            + pow(data["cuboid"]["val"][Z].asDouble(), 2.0));

        Json::Value num = data["cuboid"]["attributes"]["num"];
        int num_points  = -1;

        for(Json::Value::ArrayIndex i = 0; i < num.size(); i++) {
            // Check if the name is occlusion_level
            if(num[i]["name"].asString() == "num_points") {
                num_points = num[i]["val"].asInt();
                break;
            }
        }

        Json::Value text = data["cuboid"]["attributes"]["text"];
        std::string occlusion;
        for(Json::Value::ArrayIndex i = 0; i < text.size(); i++) {
            // Check if the name is occlusion_level
            if(text[i]["name"].asString() == "occlusion_level") {
                // Get the value of the val key as a string
                occlusion = text[i]["val"].asString();
                break;
            }
        }

        road_user->name            = data["name"].asString();
        road_user->type            = data["type"].asString();
        road_user->num_points      = num_points;
        road_user->length          = data["cuboid"]["val"][L].asDouble();
        road_user->width           = data["cuboid"]["val"][W].asDouble();
        road_user->height          = data["cuboid"]["val"][H].asDouble();
        road_user->dist2sensor     = dist2sensor;
        road_user->occlusion_level = occlusion;
    } // get_features

    void publish_pointcloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ptr)
    {
        sensor_msgs::msg::PointCloud2::SharedPtr pointcloud_msg(new sensor_msgs::msg::PointCloud2);

        pcl::toROSMsg(*cloud_ptr, *pointcloud_msg);

        // Set the frame id and timestamp
        pointcloud_msg->header.frame_id = frame_id_;
        pointcloud_msg->header.stamp    = this->now();

        // Publish the point cloud messages
        pointcloud_pub_->publish(*pointcloud_msg);
    }

    std::vector<std::string> get_files_in_folder(const std::string& folder)
    {
        std::vector<std::string> files;

        // Open the folder directory
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

    int user_input_handler()
    {
        int out = 0;
        int key = cv::waitKey(0);

        if(key != -1) {
            switch(key) {
                case 27: // Escape
                    out = SHUTDOWN;
                    break;

                case 13: // Enter
                    out = STORE;
                    break;

                case 8:
                    out = IGNORE;
                    break;

                default:
                    out = IGNORE;
                    break;
            }
        }
        return out;
    }

    Json::Value read_json_file(std::string& filepath)
    {
        // Create an input file stream object
        std::ifstream ifs(filepath);

        Json::Value data;

        // Check if the file stream is open and valid
        if(ifs.is_open() && ifs.good()) {
            // Create a json reader object
            Json::Reader reader;

            // Parse the json file and store the result in data
            bool success = reader.parse(ifs, data);

            // Check if the parsing was successful
            if(!success) {
                RCLCPP_ERROR(this->get_logger(), "%s", reader.getFormattedErrorMessages().c_str());
            }

            ifs.close();
        } else {
            RCLCPP_ERROR(this->get_logger(), "Error: Cannot open or read the file.");
        }

        return data;
    }

    void write_csv(std::vector<Road_User_t>& vec, std::string& filename)
    {
        // check if the file already exists
        std::ifstream infile(filename);
        bool file_exists = infile.good();

        infile.close();

        // Create an output file stream object
        std::ofstream file(filename, std::ios::app);

        // Check if the file is opened successfully
        if(file.is_open()) {
            // set the precision of the file stream to 6 decimal places
            file << std::setprecision(6);

            if(false == file_exists) {
                file << "name,type,num_points,length,width,height,dist2sensor,occlusion_level\n";
            }

            // Loop through the vector elements
            for(auto entry : vec) {
                // Write the element to the file, followed by a comma
                file << entry.name << ","
                     << entry.type << ","
                     << entry.num_points << ","
                     << entry.length << ","
                     << entry.width << ","
                     << entry.height << ","
                     << entry.dist2sensor << ","
                     << entry.occlusion_level << "\n";
            }
            file.close();
        } else {
            RCLCPP_ERROR(this->get_logger(), "Error: Cannot open/write to csv file.");
        }
    } // write_csv

    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_pub_;

    std::vector<std::string> gt_files_;
    std::vector<std::string> pointcloud_files_;
    std::string frame_id_;
    std::string path_;
    size_t file_index_ = 0;
}; // worker_thread

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<FeatureExtractor>());
    rclcpp::shutdown();
    return 0;
}
