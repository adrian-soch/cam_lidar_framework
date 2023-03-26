/**
 * @file lidar_processing.cpp
 * @author Adrian Sochaniwsky (sochania@mcmaster.ca)
 * @brief 
 * @version 0.1
 * @date 2023-03-23
 * @todo
 *      - Test on different clouds (check for runtime issues)
 *      - time the clustering methods
 *      - visualize all clusters in diff colours via `region growing segmentation` tutorial
 *      - add data association and MOT
 *      - Use IPC by running this node in a container with the Ouster Node ?
 * @copyright Copyright (c) 2023
 * 
 */

#include "lidar_pipeline/lidar_processing.hpp"

namespace lidar_pipeline
{

void LidarProcessing::cloud_callback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr recent_cloud)
{
    // RCLCPP_INFO(this->get_logger(), "Cloud service called; getting a PointCloud2 on topic");
    auto start = std::chrono::high_resolution_clock::now();

    /*
    * TRANSFORM PointCloud2 FROM CAMERA FRAME TO WORLD FRAME
    */
    geometry_msgs::msg::TransformStamped stransform;
    try
    {
        stransform = tf_buffer_->lookupTransform(world_frame, recent_cloud->header.frame_id,
                                                    tf2::TimePointZero, tf2::durationFromSec(3));
    }
    catch (const tf2::TransformException &ex)
    {
        RCLCPP_ERROR(this->get_logger(), "%s", ex.what());
    }

    sensor_msgs::msg::PointCloud2 transformed_cloud;
    pcl_ros::transformPointCloud(world_frame, stransform, *recent_cloud, transformed_cloud);

    /*
    * CONVERT PointCloud2 ROS->PCL
    */
    pcl::PointCloud<pcl::PointXYZI> cloud;
    pcl::fromROSMsg(transformed_cloud, cloud);

    /* ========================================
    * VOXEL GRID
    * ========================================*/
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_ptr(new pcl::PointCloud<pcl::PointXYZI>(cloud));
    cloud_ops.voxel_grid_filter(cloud_ptr, voxel_leaf_size);

    /* ========================================
    * CROPBOX
    * ========================================*/
    pcl::PointCloud<pcl::PointXYZI>::Ptr crop_cloud_ptr(new pcl::PointCloud<pcl::PointXYZI>(*cloud_ptr));
    cloud_ops.crop_box_filter(crop_cloud_ptr, x_filter_min, y_filter_min, z_filter_min,
        x_filter_max, y_filter_max, z_filter_max, false);

    /* ========================================
    * STATISTICAL OUTLIER REMOVAL
    * ========================================*/
    pcl::PointCloud<pcl::PointXYZI>::Ptr stats_cloud_ptr(new pcl::PointCloud<pcl::PointXYZI>(*crop_cloud_ptr));
    cloud_ops.stats_outlier_removal(stats_cloud_ptr, 50, 1.0);
    
    /* ========================================
    * PLANE SEGEMENTATION
    * ========================================*/
    pcl::PointCloud<pcl::PointXYZI>::Ptr plane_ptr(new pcl::PointCloud<pcl::PointXYZI>(*crop_cloud_ptr));
    int ret_code = cloud_ops.ground_plane_removal(plane_ptr, plane_max_iter, plane_dist_thresh);

    if(ret_code != 0) {
        RCLCPP_WARN(this->get_logger(), "Could not estimate a planar model for the given dataset.");
    }

    /* ========================================
    * CLUSTERING
    * ========================================*/
    std::vector<pcl::PointIndices> cluster_indices;

    cloud_ops.euclidean_clustering(plane_ptr, cluster_indices,
        cluster_tol, cluster_min_size, cluster_max_size);

    std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> clusters;
    std::vector<vision_msgs::msg::Detection3D> bboxes;
    CubePoints max_min_pts;
    
    for (const auto &cluster : cluster_indices)
    {
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_cluster(new pcl::PointCloud<pcl::PointXYZI>);

        // Compute MinMax points AABB
        Eigen::Vector4f minPt{}, maxPt{};
        pcl::getMinMax3D(*plane_ptr, cluster, minPt, maxPt);

        max_min_pts.max_pts.push_back(maxPt);
        max_min_pts.min_pts.push_back(minPt);
        
        // Put each cluster into a vector
        for (const auto &idx : cluster.indices)
        {
            cloud_cluster->points.push_back((*plane_ptr)[idx]);
        }
        cloud_cluster->width = cloud_cluster->points.size();
        cloud_cluster->height = 1;
        cloud_cluster->is_dense = true;

        clusters.push_back(cloud_cluster);

        // Init and fill bboxes
        vision_msgs::msg::Detection3D d3d;
        vision_msgs::msg::BoundingBox3D bb = getOrientedBoudingBox(*cloud_cluster);

        vision_msgs::msg::BoundingBox3D bb2 = getOrientedBoudingBox2(*cloud_cluster);
        
        // Basic Size based classifier
        std::string id = simpleClassifier(bb);
        if(!id.empty()) {                        
            
            d3d.header.stamp = recent_cloud->header.stamp;
            d3d.bbox = bb;
            d3d.id = id;
            bboxes.push_back(d3d);
        }
    }
    
    /* ========================================
        * Compute Bounding Boxes
        * ========================================*/
    std::vector<geometry_msgs::msg::Point> line_list = minMax2lines(max_min_pts);

    /* ========================================
        * CONVERT PointCloud2 PCL->ROS, PUBLISH CLOUD
        * ========================================*/
    this->publishPointCloud(voxel_grid_pub_, *cloud_ptr);
    this->publishPointCloud(plane_pub_, *plane_ptr);
    this->publishPointCloud(stat_pub_, *stats_cloud_ptr);

    this->publish3DBBox(marker_pub_, line_list);
    this->publish3DBBoxOBB(marker_array_pub_, bboxes);

    auto stop = std::chrono::high_resolution_clock::now();
    auto t_ms = std::chrono::duration_cast<std::chrono::milliseconds>(stop - start);
    RCLCPP_INFO(get_logger(), "Time (msec): %ld", t_ms.count());
}

template <typename PointT>
vision_msgs::msg::BoundingBox3D LidarProcessing::getOrientedBoudingBox(const pcl::PointCloud<PointT> &cloud_cluster)
{
    Eigen::Vector4f centroid;
    pcl::compute3DCentroid(cloud_cluster, centroid);
    PointT min_pt, max_pt;
    Eigen::Vector3f center;
    pcl::getMinMax3D(cloud_cluster, min_pt, max_pt); 
    center = (max_pt.getVector3fMap() + min_pt.getVector3fMap())/2;
    
    Eigen::Matrix3f covariance;
    pcl::computeCovarianceMatrixNormalized(cloud_cluster, centroid, covariance);
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> eigen_solver(covariance, Eigen::ComputeEigenvectors);
    Eigen::Matrix3f eigenVectorsPCA = eigen_solver.eigenvectors();
    // Eigen::Vector3f eigenValuesPCA  = eigen_solver.eigenvalues();
    eigenVectorsPCA.col(2) = eigenVectorsPCA.col(0).cross(eigenVectorsPCA.col(1));
    eigenVectorsPCA.col(0) = eigenVectorsPCA.col(1).cross(eigenVectorsPCA.col(2));
    eigenVectorsPCA.col(1) = eigenVectorsPCA.col(2).cross(eigenVectorsPCA.col(0));

    Eigen::Matrix3f eigenVectorsPCA1;
    eigenVectorsPCA1.col(0) = eigenVectorsPCA.col(2);
    eigenVectorsPCA1.col(1) = eigenVectorsPCA.col(1);
    eigenVectorsPCA1.col(2) = eigenVectorsPCA.col(0);
    eigenVectorsPCA = eigenVectorsPCA1;

    Eigen::Vector3f ea = (eigenVectorsPCA).eulerAngles(2, 1, 0); //yaw pitch roll
    Eigen::AngleAxisf keep_Z_Rot(ea[0], Eigen::Vector3f::UnitZ());
    Eigen::Affine3f transform = Eigen::Affine3f::Identity();
    transform.translate(center);
    transform.rotate(keep_Z_Rot); // radians
    
    typename pcl::PointCloud<PointT>::Ptr transformedCloud(new pcl::PointCloud<PointT>);
    pcl::transformPointCloud(cloud_cluster, *transformedCloud, transform.inverse());

    PointT min_pt_T, max_pt_T;
    pcl::getMinMax3D(*transformedCloud, min_pt_T, max_pt_T);
    Eigen::Vector3f center_new = (max_pt_T.getVector3fMap() + min_pt_T.getVector3fMap()) / 2;
    Eigen::Vector3f box_dim;
    box_dim = max_pt_T.getVector3fMap() - min_pt_T.getVector3fMap();
    Eigen::Affine3f transform2 = Eigen::Affine3f::Identity();
    transform2.translate(center_new);
    // Eigen::Affine3f transform3 = transform * transform2;

    const Eigen::Quaternionf bboxQ(keep_Z_Rot);

    PointT size;
    size.getArray3fMap() = max_pt_T.getArray3fMap() - min_pt_T.getArray3fMap();

    vision_msgs::msg::BoundingBox3D bb;
    bb.center.position.x = centroid[X];
    bb.center.position.y = centroid[Y];
    bb.center.position.z = centroid[Z];
    bb.center.orientation.x = bboxQ.x();
    bb.center.orientation.y = bboxQ.y();
    bb.center.orientation.z = bboxQ.z();
    bb.center.orientation.w = bboxQ.w();
    bb.size.x = size.x;
    bb.size.y = size.y;
    bb.size.z = size.z;

    return bb;
}

template <typename PointT>
vision_msgs::msg::BoundingBox3D LidarProcessing::getOrientedBoudingBox2(const pcl::PointCloud<PointT> &cloud_cluster)
{  
    pcl::MomentOfInertiaEstimation <PointT> feature_extractor;
    feature_extractor.setInputCloud(cloud_cluster);
    feature_extractor.compute();

    PointT min_point_OBB, max_point_OBB, position_OBB;
    Eigen::Matrix3f rotational_matrix_OBB;
    feature_extractor.getOBB(min_point_OBB, max_point_OBB, position_OBB, rotational_matrix_OBB);

    Eigen::Quaternionf quat(rotational_matrix_OBB);

    vision_msgs::msg::BoundingBox3D bb;
    bb.center.position.x = position_OBB.x;
    bb.center.position.y = position_OBB.y;
    bb.center.position.z = position_OBB.z;
    bb.center.orientation.x = quat.x();
    bb.center.orientation.y = quat.y();
    bb.center.orientation.z = quat.z();
    bb.center.orientation.w = quat.w();
    bb.size.x = size.x;
    bb.size.y = size.y;
    bb.size.z = size.z;

    return bb;
}



std::string LidarProcessing::simpleClassifier(const vision_msgs::msg::BoundingBox3D bb) {
    std::string out;

    // check size from small to large
    double cluster_vol = bb.size.x * bb.size.y * bb.size.z;

    double min_human_vol = 0.5*0.5*0.5;
    double max_human_vol = 2.2*0.7*0.4;

    double min_car_vol = 1.5*1.5*3;
    double max_car_vol = 2.5*2*6;

    double min_truck_vol = 2.5*2*6.5;
    double max_truck_vol = 3*3*15;

    if(cluster_vol <= max_human_vol && cluster_vol >= min_human_vol) {
        out = "pedestrian";
    }
    else if(cluster_vol <= max_car_vol && cluster_vol >= min_car_vol) {
        out = "car";
    }
    else if(cluster_vol <= max_truck_vol && cluster_vol >= min_truck_vol) {
        out = "truck";
    }
    
    return out;
}

void LidarProcessing::getBboxColorRGBA(const std::string id, std_msgs::msg::ColorRGBA *out)
{
    // take in the id number
    // return colour via pointer based on id/object class
    if(id == "pedestrian") {
        out->r = 1.f;
        out->g = 0.f;
        out->b = 0.f; 
    }
    else if(id == "car"){
        out->r = 0.f;
        out->g = 1.f;
        out->b = 0.f; 
    }
    else if(id == "truck"){
        out->r = 0.f;
        out->g = 0.f;
        out->b = 1.f; 
    }
    else {
        out->r = 0.f;
        out->g = 0.f;
        out->b = 0.f; 
    }
}

void LidarProcessing::publishPointCloud(rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher,
                        pcl::PointCloud<pcl::PointXYZI> point_cloud)
{
    sensor_msgs::msg::PointCloud2::SharedPtr pc2_cloud(new sensor_msgs::msg::PointCloud2);
    pcl::toROSMsg(point_cloud, *pc2_cloud);
    pc2_cloud->header.frame_id = world_frame;
    pc2_cloud->header.stamp = this->get_clock()->now();
    publisher->publish(*pc2_cloud);
}

void LidarProcessing::publish3DBBox(rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr publisher,
    const std::vector<geometry_msgs::msg::Point>& line_list)
{
    visualization_msgs::msg::Marker::SharedPtr bboxes(new visualization_msgs::msg::Marker);
    bboxes->header.frame_id = world_frame;
    bboxes->header.stamp = this->get_clock()->now();
    bboxes->type = visualization_msgs::msg::Marker::LINE_LIST;
    bboxes->action = visualization_msgs::msg::Marker::ADD;
    bboxes->points = line_list;
    bboxes->scale.x = 0.06;
    bboxes->scale.y = 0.1;
    bboxes->scale.z = 0.1;

    bboxes->color.g = 1.0;
    bboxes->color.a = 1.0;

    publisher->publish(*bboxes);
}

void LidarProcessing::publish3DBBoxOBB(rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr publisher,
    const std::vector<vision_msgs::msg::Detection3D>& bboxes)
{
    visualization_msgs::msg::MarkerArray marker_array;
    int idx = 0;
    for(auto c : bboxes) {
        visualization_msgs::msg::Marker marker;
        marker.id = idx++;
        marker.header.frame_id = world_frame;
        marker.header.stamp = this->get_clock()->now();
        marker.type = visualization_msgs::msg::Marker::CUBE;
        marker.action = visualization_msgs::msg::Marker::ADD;

        marker.pose = c.bbox.center; // Sets position and orientation
        marker.scale = c.bbox.size;  // Set w,l,h
        
        std_msgs::msg::ColorRGBA rgba;
        getBboxColorRGBA(c.id, &rgba); 
        marker.color = rgba;
        marker.color.a = 0.4;   // Set alpha so we can see underlying points

        marker_array.markers.push_back(marker);
    }
    publisher->publish(marker_array);
}

void LidarProcessing::publishDetections(rclcpp::Publisher<vision_msgs::msg::Detection3DArray>::SharedPtr publisher,
    const std::vector<vision_msgs::msg::Detection3D>& detections)
{
    vision_msgs::msg::Detection3DArray det_array;
    det_array.detections = detections;
    det_array.header.frame_id = world_frame;
    det_array.header.stamp = this->get_clock()->now();
    
    publisher->publish(det_array);
}

std::vector<geometry_msgs::msg::Point> LidarProcessing::minMax2lines(CubePoints &max_min_pts)
{
    std::vector<geometry_msgs::msg::Point> out;
    geometry_msgs::msg::Point p0, p1, v1, v2, v3, v4, v5, v6, v7, v8;

    auto i = max_min_pts.max_pts.begin();
    auto j = max_min_pts.min_pts.begin();
    while (i != max_min_pts.max_pts.end() and j != max_min_pts.min_pts.end())
    {
        auto p1 = *i++;
        auto p2 = *j++;

        // Create 8 vertices of a cube
        v1.x = p1[X];
        v1.y = p1[Y];
        v1.z = p1[Z];

        v2.x = p2[X];
        v2.y = p1[Y];
        v2.z = p1[Z];

        v3.x = p2[X];
        v3.y = p2[Y];
        v3.z = p1[Z];

        v4.x = p1[X];
        v4.y = p2[Y];
        v4.z = p1[Z];

        v5.x = p1[X];
        v5.y = p1[Y];
        v5.z = p2[Z];

        v6.x = p2[X];
        v6.y = p1[Y];
        v6.z = p2[Z];

        v7.x = p2[X];
        v7.y = p2[Y];
        v7.z = p2[Z];

        v8.x = p1[X];
        v8.y = p2[Y];
        v8.z = p2[Z];

        // Append points in pairs for marker LINE_LIST to create cube edges
        out.insert(out.end(), {v1,v2, v2,v3, v1,v4, v3,v4,
                                v1,v5, v3,v7, v4,v8, v2,v6,
                                v5,v8, v7,v8, v5,v6, v6,v7});
    }
    return out;
}

} // end namespace lidar_pipeline