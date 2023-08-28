#include <filesystem>
#include <fstream>
#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <string>
#include <vector>

// Use the std namespace
using namespace std;

// Define a function to read a .pcd file with the PCL library and return a point cloud pointer
pcl::PointCloud<pcl::PointXYZI>::Ptr read_pcd(const string& filename)
{
    // Create an empty point cloud pointer
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI> );

    // Load the .pcd file into the point cloud
    if(pcl::io::loadPCDFile<pcl::PointXYZI>(filename, *cloud) == -1) {
        // Print an error message if the file cannot be loaded
        cerr << "Couldn't read file " << filename << endl;
    }

    // Return the point cloud pointer
    return cloud;
}

// Define a function to write a point cloud as a xyzi text file
void write_xyzi(const string& filename, const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud)
{
    // Open the file for writing
    ofstream file(filename);

    // Check if the file is open
    if(file.is_open()) {
        // Loop through each point in the point cloud
        for(const auto& point : cloud->points) {
            // Write the x, y, z, and i values of the point separated by whitespace
            file << point.x << " " << point.y << " " << point.z << " " << point.intensity << endl;
        }

        // Close the file
        file.close();
    }
}

// Define the main function
int main(int argc, char** argv)
{
    // Check if a directory is given as an argument
    if(argc < 2) {
        // Print an error message and exit
        cerr << "Usage: " << argv[0] << " <directory>" << endl;
        return -1;
    }

    // Get the directory name from the argument
    string dir = argv[1];

    // Loop through each file in the directory
    for(const auto& entry : filesystem::directory_iterator(dir)) {
        // Get the file path and name
        string path = entry.path().string();
        string name = entry.path().filename().string();

        // Check if the file has a .pcd extension
        if(entry.path().extension() == ".pcd") {
            // Read the .pcd file with the PCL library and get the point cloud pointer
            pcl::PointCloud<pcl::PointXYZI>::Ptr cloud = read_pcd(path);

            // Change the extension of the file name to .xyzi

            path.replace(path.find(".pcd"), 4, ".txt");

            // Write the point cloud as a xyzi text file with the same name
            std::cout << path << std::endl;
            write_xyzi(path, cloud);
        }
    }

    // Return 0 to indicate success
    return 0;
} // main
