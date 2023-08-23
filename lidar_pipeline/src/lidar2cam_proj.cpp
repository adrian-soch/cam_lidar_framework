/**
 * @file lidar2cam_proj.cpp
 * @author Adrian Sochaniwsky (sochania@mcmaster.ca)
 * @brief
 * @version 0.1
 * @date 2023-06-19
 *
 * @copyright Copyright (c) 2023
 *
 */

#include <rclcpp/rclcpp.hpp>

#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>

#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <vision_msgs/msg/detection2_d_array.hpp>
#include <vision_msgs/msg/detection3_d_array.hpp>

#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl_conversions/pcl_conversions.h>

#include <cv_bridge/cv_bridge.h>

using namespace message_filters;

class Lidar2CameraProjector : public rclcpp::Node
{
public:
    Lidar2CameraProjector() : Node("l2c_proj")
    {
        RCLCPP_INFO(this->get_logger(), "Getting parameters");

        // Get Topics
        this->declare_parameter("lidar_track_topic", "/lidar_proc/tracks");
        this->declare_parameter("cam_track_topic", "/image_proc/tracks");
        this->declare_parameter("cam_result_topic", "/image_proc/result");

        this->get_parameter("lidar_track_topic", lidar_track_topic);
        this->get_parameter("cam_track_topic", cam_track_topic);
        this->get_parameter("cam_result_topic", cam_result_topic);

        // Get transforms
        this->declare_parameter<std::vector<double> >("lidar2cam_extrinsic.translation", { 0.0, 0.0508, 0.0502 });
        this->declare_parameter<std::vector<double> >("lidar2cam_extrinsic.rotation", { 0.0, 1.0, 0.0,
                                                                                        0.0, 0.0, -1.0,
                                                                                        -1.0, 0.0, 0.0 });
        this->declare_parameter<std::vector<double> >("lidarData2lidarSensor_extrinsic.translation", { 0.0, 0.0,
                                                                                                       0.03618 });
        this->declare_parameter<std::vector<double> >("lidarData2lidarSensor_extrinsic.rotation", { -1.0, 0.0, 0.0,
                                                                                                    0.0, -1.0, 0.0,
                                                                                                    0.0, 0.0, 1.0 });
        this->declare_parameter<std::vector<double> >("camera_matrix.translation", { 0.0, 0.0, 0.0 });
        this->declare_parameter<std::vector<double> >("camera_matrix.rotation", { 1199.821557, 0.000000, 960.562236,
                                                                                  0.000000, 1198.033465, 551.675808,
                                                                                  0.000000, 0.000000, 1.000000 });

        this->get_parameter("lidar2cam_extrinsic.translation", lidar2cam_translation);
        this->get_parameter("lidar2cam_extrinsic.rotation", lidar2cam_rotation);
        this->get_parameter("lidarData2lidarSensor_extrinsic.translation", lidarData2lidarSensor_translation);
        this->get_parameter("lidarData2lidarSensor_extrinsic.rotation", lidarData2lidarSensor_rotation);
        this->get_parameter("camera_matrix.translation", camera_matrix_translation);
        this->get_parameter("camera_matrix.rotation", camera_matrix_rotation);


        // Create subscribers for the two topics
        sub_image_     = std::make_shared<Subscriber<sensor_msgs::msg::Image> >(this, cam_result_topic);
        sub_detection_ = std::make_shared<Subscriber<vision_msgs::msg::Detection3DArray> >(this, lidar_track_topic);

        // Create publisher
        pub_      = this->create_publisher<sensor_msgs::msg::Image>("image_proc/projected_dets", 1);
        proj_pub_ = this->create_publisher<vision_msgs::msg::Detection2DArray>("image_proc/lidar_track_2D", 1);

        // Create a sync policy using the approximate time synchronizer
        sync_ = std::make_shared<Synchronizer<sync_policies::ApproximateTime<sensor_msgs::msg::Image,
            vision_msgs::msg::Detection3DArray> > >(50);
        sync_->connectInput(*sub_image_, *sub_detection_);

        // Register a callback for the synchronized messages
        sync_->registerCallback(&Lidar2CameraProjector::callback, this);

        lidar2cam = param2Transform(lidar2cam_rotation, lidar2cam_translation);
        lidarData2lidarSensor = param2Transform(lidarData2lidarSensor_rotation,
            lidarData2lidarSensor_translation);
        cam_mat = param2Transform(camera_matrix_rotation, camera_matrix_translation);

        sensor2world = Eigen::Affine3f::Identity();
        sensor2world.translation() << 0.0, 0.0, 12.0;
        sensor2world.rotate(Eigen::Quaternionf(0.9795752, 0.0, 0.2010779, 0.0));
    }

private:
    void callback(const sensor_msgs::msg::Image::ConstSharedPtr & image,
      const vision_msgs::msg::Detection3DArray::ConstSharedPtr  & lidar_track)
    {
        auto start = std::chrono::high_resolution_clock::now();

        RCLCPP_INFO(this->get_logger(), "Received an image and a detection");

        cv_bridge::CvImagePtr cv_ptr;
        try
        {
            cv_ptr = cv_bridge::toCvCopy(image);
        }
        catch(cv_bridge::Exception& e)
        {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
            return;
        }

        // Use the inverse of the of lidar2ground transformed used furher up the pipeline
        // If we dont do this the points dont align with the image.
        Eigen::Affine3f proj_matrix = cam_mat * lidar2cam * lidarData2lidarSensor * sensor2world.inverse();
        // Eigen::Affine3f transformation_matrix = sensor2world.inverse(); // USE THIS FOR EXTERNAL PROJECTION CODE


        std::vector<pcl::PointCloud<pcl::PointXYZ> > clouds;
        for(auto det: lidar_track->detections) {
            pcl::PointCloud<pcl::PointXYZ> cloud = center_size2points(det.bbox);
            clouds.push_back(cloud);
        }

        pcl::PointCloud<pcl::PointXYZ> final_cloud, proj_cloud;
        for(auto c: clouds) {
            final_cloud += c;
        }
        proj_cloud = final_cloud;

        // Transform the 3xN matrix
        pcl::transformPointCloud(proj_cloud, proj_cloud, proj_matrix);

        // Project to 2D, divide x, y by z
        for(size_t i = 0; i < proj_cloud.size(); i++) {
            pcl::PointXYZ p = proj_cloud.points[i];
            cv::Point2f p2d(p.x / p.z, p.y / p.z);
            cv::circle(cv_ptr->image, p2d, 6, CV_RGB(255, 0, 0), -1);
        }

        // Convert the OpenCV image to a ROS image message using cv_bridge
        sensor_msgs::msg::Image::SharedPtr processed_msg = cv_ptr->toImageMsg();
        processed_msg->header = image->header;

        // Publish the processed image
        pub_->publish(*processed_msg);
        // proj_pub_->publish();

        auto stop = std::chrono::high_resolution_clock::now();
        auto t_ms = std::chrono::duration_cast<std::chrono::milliseconds>(stop - start);
        RCLCPP_INFO(get_logger(), "Time (msec): %ld", t_ms.count());
    } // callback

    pcl::PointCloud<pcl::PointXYZ> center_size2points(vision_msgs::msg::BoundingBox3D bbox)
    {
        // Get 8 vertices relative to each other
        float x_len = bbox.size.x / 2.0;
        float y_len = bbox.size.y / 2.0;
        float z_len = bbox.size.z / 2.0;

        pcl::PointCloud<pcl::PointXYZ> cloud;

        cloud.width  = 8;
        cloud.height = 1;
        cloud.points.push_back(pcl::PointXYZ(-x_len, -y_len, -z_len));
        cloud.points.push_back(pcl::PointXYZ(-x_len, -y_len, +z_len));
        cloud.points.push_back(pcl::PointXYZ(-x_len, +y_len, -z_len));
        cloud.points.push_back(pcl::PointXYZ(-x_len, +y_len, +z_len));
        cloud.points.push_back(pcl::PointXYZ(+x_len, -y_len, -z_len));
        cloud.points.push_back(pcl::PointXYZ(+x_len, -y_len, +z_len));
        cloud.points.push_back(pcl::PointXYZ(+x_len, +y_len, -z_len));
        cloud.points.push_back(pcl::PointXYZ(+x_len, +y_len, +z_len));

        geometry_msgs::msg::Pose pose;
        pose = bbox.center;

        // Orient the cube
        Eigen::Affine3f transform = pose2Transform(pose);
        pcl::transformPointCloud(cloud, cloud, transform);

        return cloud;
    }

    Eigen::Affine3f param2Transform(std::vector<double> rot, std::vector<double> trans)
    {
        Eigen::Affine3f output = Eigen::Affine3f::Identity();

        // Set rotational component
        output.linear() << rot[0], rot[1], rot[2], rot[3], rot[4], rot[5], rot[6], rot[7], rot[8];

        // Set the translation component
        output.translation() << trans[0], trans[1], trans[2];

        return output;
    }

    Eigen::Affine3f pose2Transform(geometry_msgs::msg::Pose pose)
    {
        Eigen::Affine3f output = Eigen::Affine3f::Identity();

        output.rotate(Eigen::Quaternionf(pose.orientation.w, pose.orientation.x, pose.orientation.y,
          pose.orientation.z));
        output.translation() << pose.position.x, pose.position.y, pose.position.z;
        return output;
    }

    /*
     * Parameters
     */
    std::string lidar_track_topic;
    std::string cam_track_topic;
    std::string cam_result_topic;

    Eigen::Affine3f lidar2cam, lidarData2lidarSensor, cam_mat, sensor2world;

    std::vector<double> lidar2cam_translation;
    std::vector<double> lidar2cam_rotation;
    std::vector<double> lidarData2lidarSensor_translation;
    std::vector<double> lidarData2lidarSensor_rotation;
    std::vector<double> camera_matrix_translation;
    std::vector<double> camera_matrix_rotation;

    std::shared_ptr<Subscriber<sensor_msgs::msg::Image> > sub_image_;
    std::shared_ptr<Subscriber<vision_msgs::msg::Detection3DArray> > sub_detection_;
    std::shared_ptr<Synchronizer<sync_policies::ApproximateTime<sensor_msgs::msg::Image,
      vision_msgs::msg::Detection3DArray> > > sync_;

    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_;
    rclcpp::Publisher<vision_msgs::msg::Detection2DArray>::SharedPtr proj_pub_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<Lidar2CameraProjector>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
