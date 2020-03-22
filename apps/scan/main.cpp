
#include <string>

#include "OpenLiDAR.h"

#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>

#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/features/normal_3d.h>


float leaf_size = 0.01f; // m

// Main program
//============================================================================
int main(int argc, char **argv){

    OpenLiDAR scanner;
    if (scanner.connect()) {
        std::cout << "Start Scanning" << std::endl;
        // Scan 75% loop at half speed
        std::vector<glm::vec3> points = scanner.scan(0.75, .5);

        std::cout << "Got " << points.size() << " points" << std::endl;
        if (points.size() > 0) {

            // Declare Cloud data
            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud( new pcl::PointCloud<pcl::PointXYZ> );
            cloud->width    = points.size();
            cloud->height   = 1;
            cloud->is_dense = true;
            cloud->points.resize (points.size());

            for (size_t i = 0; i < points.size(); i++) {
                cloud->points[i].x = points[i].x;
                cloud->points[i].y = points[i].y;
                cloud->points[i].z = points[i].z;
            }
            
            pcl::VoxelGrid<pcl::PointXYZ> vg_filter;
            vg_filter.setInputCloud (cloud);
            vg_filter.setLeafSize (leaf_size, leaf_size, leaf_size);
            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered( new pcl::PointCloud<pcl::PointXYZ> );
            vg_filter.filter(*cloud_filtered);

            // // Normal estimation*
            pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> n;
            pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal>);
            pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
            tree->setInputCloud (cloud_filtered);
            n.setInputCloud (cloud_filtered);
            n.setSearchMethod (tree);
            n.setKSearch (10);
            n.compute (*normals);

            // Concatenate the XYZ and normal fields*
            pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals (new pcl::PointCloud<pcl::PointNormal>);
            pcl::concatenateFields (*cloud_filtered, *normals, *cloud_with_normals);

            std::cout << "Saving points" << std::endl;
            pcl::io::savePLYFile ("point_cloud.ply", *cloud_with_normals, false);
        }

        std::cout << "Reset scanner" << std::endl;
        scanner.reset();
        scanner.disconnect();
    }

    return 0;
}