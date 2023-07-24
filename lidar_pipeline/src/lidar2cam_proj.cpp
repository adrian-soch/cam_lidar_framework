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
#include <vision_msgs/msg/detection3_d_array.hpp>

#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl_conversions/pcl_conversions.h>

#include <opencv2/calib3d.hpp>
#include <opencv2/core/eigen.hpp>
#include <opencv2/imgproc/imgproc.hpp>

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
        pub_    = this->create_publisher<sensor_msgs::msg::Image>("image_proc/projected_dets", 1);
        pc_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("lidar_proc/projected_dets_debug", 1);

        // Create a sync policy using the approximate time synchronizer
        sync_ = std::make_shared<Synchronizer<sync_policies::ApproximateTime<sensor_msgs::msg::Image,
            vision_msgs::msg::Detection3DArray> > >(50);
        sync_->connectInput(*sub_image_, *sub_detection_);

        // Register a callback for the synchronized messages
        sync_->registerCallback(&Lidar2CameraProjector::callback, this);
    }

private:
    void callback(const sensor_msgs::msg::Image::ConstSharedPtr & image,
      const vision_msgs::msg::Detection3DArray::ConstSharedPtr  & lidar_track)
    {
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

        /**
         * @brief Now that converted 3dbbox points match the processed input(with lag)
         *  We need the 3dbbox to match the original input (before cam2ground) [DONE]
         *
         * We need to perform the projection calcs manually
         */

        Eigen::Affine3f lidar2cam = param2Transform(lidar2cam_rotation, lidar2cam_translation);
        Eigen::Affine3f lidarData2lidarSensor = param2Transform(lidarData2lidarSensor_rotation,
            lidarData2lidarSensor_translation);
        Eigen::Affine3f cam_mat = param2Transform(camera_matrix_rotation, camera_matrix_translation);

        Eigen::Affine3f sensor2world = Eigen::Affine3f::Identity();
        sensor2world.translation() << 0.0, 0.0, 12.0;
        sensor2world.rotate(Eigen::Quaternionf(0.0, 0.2010779, 0.0, 0.9795752));

        // Use the inverse of the of lidar2ground transformed used furher up the pipeline
        // If we dont do this the points dont align with the image.
        // Eigen::Affine3f transformation_matrix = cam_mat * lidar2cam * lidarData2lidarSensor * sensor2world.inverse();
        Eigen::Affine3f transformation_matrix = cam_mat * lidar2cam * sensor2world.inverse();

        std::vector<pcl::PointCloud<pcl::PointXYZ> > clouds;
        for(auto det: lidar_track->detections) {
            pcl::PointCloud<pcl::PointXYZ> cloud = center_size2points(det.bbox);
            clouds.push_back(cloud);
        }

        pcl::PointCloud<pcl::PointXYZ> final_cloud;
        for(auto c: clouds) {
            final_cloud += c;
        }

        // Transform the 3xN matrix
        pcl::transformPointCloud(final_cloud, final_cloud, transformation_matrix);

        for(size_t i = 0; i < final_cloud.size(); i++) {
            pcl::PointXYZ p = final_cloud.points[i];

            if(abs(p.z) < 0.000001) {
                continue;
            }

            cv::Point2f p2d(p.x / p.z, p.y / p.z);

            cv::circle(cv_ptr->image, p2d, 6, CV_RGB(255, 0, 0), -1);
            RCLCPP_INFO(this->get_logger(), "X: %f, Y: %f", p.x, p.y);
        }


        // Convert the OpenCV image to a ROS image message using cv_bridge
        sensor_msgs::msg::Image::SharedPtr processed_msg = cv_ptr->toImageMsg();
        processed_msg->header = image->header;

        // Publish the processed image
        pub_->publish(*processed_msg);

        sensor_msgs::msg::PointCloud2::SharedPtr pc2_cloud(new sensor_msgs::msg::PointCloud2);
        pcl::toROSMsg(final_cloud, *pc2_cloud);
        pc2_cloud->header = lidar_track->header;
        pc_pub_->publish(*pc2_cloud);
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
        // RCLCPP_INFO(
        //     this->get_logger(), "%f, %f", pose.position.z, pose.orientation.z);

        // Orient the cube
        Eigen::Affine3f transform = pose2Transform(pose);
        pcl::transformPointCloud(cloud, cloud, transform);

        return cloud;
    }

    Eigen::Affine3f param2Transform(std::vector<double> rot, std::vector<double> trans)
    {
        Eigen::Affine3f output = Eigen::Affine3f::Identity();

        // Reshape/map vector to a 3x3 Eigen rotation matrix
        Eigen::Map<Eigen::Matrix3d> matrix(rot.data(), 3, 3);
        Eigen::Matrix3f matrix_f = matrix.cast<float>();

        output.linear() = matrix_f;

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
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pc_pub_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<Lidar2CameraProjector>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
