#include <pcl/io/pcd_io.h>
#include <pcl/console/print.h>
#include <pcl/console/parse.h>
#include <pcl/console/time.h>
#include "pcl/io/pcd_io.h"
#include "pcl/common/common.h"

#include <iostream>
#include <fstream>
#include <string>

void printUsage(const char *progName)
{
    std::cout << "\nUse: " << progName << " <file> -o <output dir>" << std::endl
              << "Support: .pcd" << std::endl
              << "[q] to exit" << std::endl;
}

int main(int argc, char **argv)
{
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>());
    std::vector<int> filenames;
    bool file_is_pcd = false;

    if (argc < 4 or argc > 4)
    {
        printUsage(argv[0]);
        return -1;
    }

    pcl::console::TicToc tt;
    pcl::console::print_highlight("Loading ...");

    if (filenames.size() <= 0)
    {
        filenames = pcl::console::parse_file_extension_argument(argc, argv, ".pcd");
        if (filenames.size() == 1)
        {
            file_is_pcd = true;
        }
    }
    else
    {
        printUsage(argv[0]);
        return -1;
    }

    std::string output_dir;
    if (pcl::console::parse_argument(argc, argv, "-o", output_dir) == -1)
    {
        PCL_ERROR("Need an output directory! Please use <input cloud> -o <output dir>\n");
        return (-1);
    }

    if (file_is_pcd)
    {
        if (pcl::io::loadPCDFile(argv[filenames[0]], *cloud) < 0)
        {
            std::cout << "Error loading point cloud " << argv[filenames[0]] << "\n";
            printUsage(argv[0]);
            return -1;
        }
        pcl::console::print_info("\nFound pcd file.\n");
        pcl::console::print_info("Done, (");
        pcl::console::print_value("%g", tt.toc());
        pcl::console::print_info(" ms : ");
        pcl::console::print_value("%d", cloud->size());
        pcl::console::print_info(" points)\n");
    }
    else
    {
        printUsage(argv[0]);
        return -1;
    }

    cloud->width = (int)cloud->points.size();
    cloud->height = 1;
    cloud->is_dense = true;

    std::ofstream fout;

    std::string str1 = output_dir;
    str1 += "/txt_cloud.xyzi";
    fout.open(str1.c_str());

    for (int i = 0; i < cloud->points.size(); ++i)
    {
        fout << cloud->points[i].x << " " << cloud->points[i].y << " " << cloud->points[i].z << " " << cloud->points[i].intensity << std::endl;
    }

    fout.close();

    pcl::console::print_info("Data saved");
    std::cout << std::endl;

    return 0;
}