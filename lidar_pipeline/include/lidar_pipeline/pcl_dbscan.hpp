#ifndef PCL_DBSCAN_HPP
#define PCL_DBSCAN_HPP

#include <vector>

#include <pcl/search/kdtree.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_types.h>

template<typename PointType>
class DBSCAN {
public:
    using PointCloud         = pcl::PointCloud<PointType>;
    using PointCloudPtr      = typename PointCloud::Ptr;
    using PointCloudConstPtr = typename PointCloud::ConstPtr;

    using Tree    = pcl::search::KdTree<PointType>;
    using TreePtr = typename Tree::Ptr;

    void set_params(float eps, size_t min_pts)
    {
        eps_    = eps;
        minPts_ = min_pts;
    }

    /**
     * Runs the DBSCAN algorithm on the given point cloud.
     *
     * @param cloud The point cloud to cluster.
     * @return A vector of PointIndices, where each PointIndices object contains the indices of the points in a cluster.
     */
    std::vector<pcl::PointIndices> run(PointCloudPtr cloud)
    {
        std::vector<int> labels(cloud->points.size(), -1);
        int cluster = 0;
        pcl::KdTreeFLANN<PointType> kdtree;

        kdtree.setInputCloud(cloud);

        for(size_t i = 0; i < cloud->points.size(); i++) {
            // Skip if point is already classified
            if(labels[i] != -1) continue;

            // Find the neighbors of the point
            std::vector<int> neighbors = regionQuery(cloud, kdtree, i);

            // If the number of neighbors is less than minPts, mark as noise
            if(neighbors.size() < minPts_) {
                labels[i] = 0;
            } else {
                // Otherwise, start a new cluster and expand it
                cluster++;
                expandCluster(cloud, kdtree, labels, i, neighbors, cluster);
            }
        }

        // Convert labels to a vector of PointIndices
        std::vector<pcl::PointIndices> clusters(cluster);
        for(size_t i = 0; i < labels.size(); i++) {
            if(labels[i] > 0) {
                clusters[labels[i] - 1].indices.push_back(i);
            }
        }
        return clusters;
    } // run

private:
    double eps_    = 0.5;
    size_t minPts_ = 10;

    /**
     * Finds the neighbors of a point within a radius of eps.
     *
     * @param cloud The point cloud.
     * @param kdtree The KD-tree for the point cloud.
     * @param p The index of the point to find neighbors for.
     * @return A vector of indices of the neighboring points.
     */
    std::vector<int> regionQuery(PointCloudPtr cloud, pcl::KdTreeFLANN<PointType>& kdtree, int p)
    {
        std::vector<int> indices;
        std::vector<float> distances;

        kdtree.radiusSearch(cloud->points[p], eps_, indices, distances);
        return indices;
    }

    /**
     * Expands a cluster by recursively adding density-reachable points.
     *
     * @param cloud The point cloud.
     * @param kdtree The KD-tree for the point cloud.
     * @param labels The vector of labels for each point in the cloud.
     * @param p The index of the point to start expanding from.
     * @param neighbors The neighbors of point p.
     * @param cluster The current cluster label.
     */
    void expandCluster(PointCloudPtr cloud, pcl::KdTreeFLANN<PointType>& kdtree,
      std::vector<int>& labels, int p,
      std::vector<int>& neighbors, int cluster)
    {
        labels[p] = cluster;
        for(size_t i = 0; i < neighbors.size(); i++) {
            int q = neighbors[i];

            // If q is unclassified, label it as part of cluster and find its neighbors
            if(labels[q] == 0) {
                labels[q] = cluster;
            }
            if(labels[q] != -1) continue;
            labels[q] = cluster;
            std::vector<int> qNeighbors = regionQuery(cloud, kdtree, q);
            if(qNeighbors.size() >= minPts_) {
                neighbors.insert(neighbors.end(), qNeighbors.begin(), qNeighbors.end());
            }
        }
    }
};

#endif // ifndef PCL_DBSCAN_HPP
