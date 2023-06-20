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
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <sensor_msgs/msg/image.hpp>
#include <vision_msgs/msg/detection3_d_array.hpp>

#include <pcl/common/transforms.h>

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
        this->declare_parameter<std::vector<double>>("lidar2cam_extrinsic.translation", {0.0, 0.0, 0.0});
        this->declare_parameter<std::vector<double>>("lidar2cam_extrinsic.rotation", {1, 0, 0, 0, 1, 0, 0, 0, 1});
        this->declare_parameter<std::vector<double>>("lidarData2lidarSensor_extrinsic.translation", {0.0, 0.0, 0.0});
        this->declare_parameter<std::vector<double>>("lidarData2lidarSensor_extrinsic.rotation", {1, 0, 0, 0, 1, 0, 0, 0, 1});
        this->declare_parameter<std::vector<double>>("camera_matrix.translation", {0.0, 0.0, 0.0});
        this->declare_parameter<std::vector<double>>("camera_matrix.rotation", {1, 0, 0, 0, 1, 0, 0, 0, 1});

        this->get_parameter("lidar2cam_extrinsic.translation", lidar2cam_translation);
        this->get_parameter("lidar2cam_extrinsic.rotation", lidar2cam_rotation);
        this->get_parameter("lidarData2lidarSensor_extrinsic.translation", lidarData2lidarSensor_translation);
        this->get_parameter("lidarData2lidarSensor_extrinsic.rotation", lidarData2lidarSensor_rotation);
        this->get_parameter("camera_matrix.translation", camera_matrix_translation);
        this->get_parameter("camera_matrix.rotation", camera_matrix_rotation);


        // Create subscribers for the two topics
        sub_image_ = std::make_shared<Subscriber<sensor_msgs::msg::Image>>(this, cam_result_topic);
        sub_detection_ = std::make_shared<Subscriber<vision_msgs::msg::Detection3DArray>>(this, lidar_track_topic);

        // Create a sync policy using the approximate time synchronizer
        sync_ = std::make_shared<Synchronizer<sync_policies::ApproximateTime<sensor_msgs::msg::Image, vision_msgs::msg::Detection3DArray>>>(101);
        sync_->connectInput(*sub_image_, *sub_detection_);

        // Register a callback for the synchronized messages
        sync_->registerCallback(&Lidar2CameraProjector::callback, this);
    }

private:
    void callback(const sensor_msgs::msg::Image::ConstSharedPtr& image, const vision_msgs::msg::Detection3DArray::ConstSharedPtr& lidar_track)
    {
        RCLCPP_INFO(this->get_logger(), "Received an image and a detection");

        /**
         * @todo:
         * From python:
         * 
         * 
         * # Return x,y,z,1
            xyz1 = np.hstack([pc[:, :3], np.ones((pc.shape[0], 1), dtype=np.float32)])

            # from dataFrame -> sensorFrame -> cameraFrame
            #       (3*4) @ (4*4) @ (4*N) = (3*N)
            temp = self.l2c_mat @  self.ld2ls_mat @ xyz1.T
            
            '''
            Project pc to img
            ###################################################
            '''
            # Get projection on image plane
            # (3*3) @ (3*N) = (3*N)
            px_proj = (self.camera_mat @ temp)

            # Get depth
            depth = px_proj[2 :].T

            # Divide by depth to get 2-D projection
            px_proj = (px_proj[:2,:] / px_proj[2 :]).T
         * - Restrict points to camera FoV
            - Colour points based on range
        */
        Eigen::Affine3f lidar2cam = param2Transform(lidar2cam_rotation, lidar2cam_translation);
        Eigen::Affine3f lidarData2lidarSensor = param2Transform(lidarData2lidarSensor_rotation,
                lidarData2lidarSensor_translation);
        Eigen::Affine3f camera_matrix = param2Transform(camera_matrix_rotation, camera_matrix_translation);
       
        // 1. Take [x,y,z] and append such that it becomes [x,y,z,1]
        // 2. cam_mat @ lidar2cam @ lidarDara2lidarSensor @ xyz1.T
        // 3. reserve z values (depth)
        // 4. get projection by divind [x,y]/z
        // 5. crop points to retain camera fov
        // 6. Draw points on image + publish

    }

    Eigen::Affine3f param2Transform(std::vector<double> rot, std::vector<double> trans) {
        Eigen::Affine3f output = Eigen::Affine3f::Identity();

        // Reshape/map vector to a 3x3 Eigen rotation matrix
        output.rotate(Eigen::Map<Eigen::Matrix3f>((float*)rot.data()));

        // Set the translation component
        output.translation() << trans[0], trans[1], trans[2];

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


    std::shared_ptr<Subscriber<sensor_msgs::msg::Image>> sub_image_;
    std::shared_ptr<Subscriber<vision_msgs::msg::Detection3DArray>> sub_detection_;
    std::shared_ptr<Synchronizer<sync_policies::ApproximateTime<sensor_msgs::msg::Image, vision_msgs::msg::Detection3DArray>>> sync_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<Lidar2CameraProjector>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
