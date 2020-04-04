
#include "OpenLiDAR.h"
#include "tools/textOps.h"
#include "tools/fileOps.h"

#include <pcl/io/ply_io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/features/normal_3d.h>

template<typename PointT> 
inline bool savePointCloud( const std::string& _filename, 
                            const std::vector<std::string>& _formats, 
                            const pcl::PointCloud<PointT> &_cloud) 
{

    for (size_t i = 0; i < _formats.size(); i++) {
        std::string filename = getUniqueFileName(_filename, _formats[i]);
        if (_formats[i] == "ply") {
            std::cout << "Saving points as " << filename << std::endl;
            pcl::io::savePLYFile(filename, _cloud, false);
        }
        else if (_formats[i] == "pcd") {
            std::cout << "Saving points as " << filename << std::endl;
            pcl::io::savePCDFile(filename, _cloud, false);
        }
    }

    return true;
}

// Main program
//============================================================================
int main(int argc, char **argv){
    OpenLiDAR scanner;
    OpenLiDARSettings settings;

    std::string filename = "pcl";
    std::vector<std::string> formats = { "ply" };
    float toDegree = 180.0f;    // Half loop
    float atSpeed = 0.75f;      // 75% of speed
    float leaf = 0.01f;         // 0.01m -> 1cm
    bool bNormal = false;
    bool bVerbose = false;

    for (int i = 1; i < argc ; i++) {
        std::string argument = std::string(argv[i]);

        if ( std::string(argv[i]) == "--filename" ) {
            if (++i < argc)
                filename = std::string(argv[i]);
        }
        else if ( std::string(argv[i]) == "--formats" ) {
            if (++i < argc) {
                formats.clear();
                formats = split(std::string(argv[i]), ',', true);
            }
        }
        else if ( std::string(argv[i]) == "--degrees" ) {
            if (++i < argc)
                toDegree = toFloat(std::string(argv[i]));
            else
                std::cout << "Argument '" << argument << "' should be followed by a amount of degrees to turn during the scan. Default is " << toDegree << std::endl;
        }
        else if ( std::string(argv[i]) == "--speed" ) {
            if (++i < argc)
                atSpeed = toFloat(std::string(argv[i]));
            else
                std::cout << "Argument '" << argument << "' should be followed by a the speed (expressed in a number between 0.0 to 1.0) of the speed of the scan. Default is " << atSpeed << std::endl;
        }
        else if ( std::string(argv[i]) == "--leaf" ) {
            if (++i < argc)
                leaf = toFloat(std::string(argv[i]));
            else
                std::cout << "Argument '" << argument << "' should be followed by a the leaf size (expressed in meters) for the voxel grid. Default is" << leaf << std::endl;
        }
        else if ( std::string(argv[i]) == "--normals" )
            bNormal = true;
        else if ( std::string(argv[i]) == "--verbose" )
            bVerbose = true;
    }

    
    if (scanner.connect(settings, bVerbose)) {

        // Scan 75% degrees at half speed
        std::vector<glm::vec4> points = scanner.scan(toDegree, atSpeed, bVerbose);

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

            if (leaf > 0.0) {
                pcl::VoxelGrid<pcl::PointXYZ> vg_filter;
                vg_filter.setInputCloud (cloud);
                vg_filter.setLeafSize (leaf, leaf, leaf);
                pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered( new pcl::PointCloud<pcl::PointXYZ> );
                vg_filter.filter(*cloud_filtered);
                *cloud = *cloud_filtered;
            }
            
            if (bNormal) {
                std::cout << "Estimating normals" << std::endl;
                // // Normal estimation*
                pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> n;
                pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal>);
                pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
                tree->setInputCloud (cloud);
                n.setInputCloud (cloud);
                n.setSearchMethod (tree);
                n.setKSearch (10);
                n.compute (*normals);

                // Concatenate the XYZ and normal fields*
                pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals (new pcl::PointCloud<pcl::PointNormal>);
                pcl::concatenateFields (*cloud, *normals, *cloud_with_normals);

                savePointCloud(filename, formats, *cloud_with_normals);
            }
            else {
                savePointCloud(filename, formats, *cloud);
            }
        }

        scanner.reset(bVerbose);
        scanner.disconnect();
    }

    return 0;
}
