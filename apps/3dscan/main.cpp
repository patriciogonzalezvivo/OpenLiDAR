
#include "OpenLiDAR.h"
#include "mary/tools/textOps.h"
#include "../common/fileOps.h"
#include "../common/colorOps.h"
#include "../common/io.h"

#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/features/normal_3d.h>

// Main program
//============================================================================
int main(int argc, char **argv){
    OpenLiDAR scanner;
    OpenLiDARSettings settings;

    std::string filename = "pcl.ply";
    std::vector<std::string> formats;
    float toDegree = 359.0f;    // Full loop
    float atSpeed = 0.75f;      // 75% of speed
    float voxel = 0.0f;
    bool bNormal = false;
    bool bVerbose = false;

    for (int i = 1; i < argc ; i++) {
        std::string argument = std::string(argv[i]);

        if ( std::string(argv[i]) == "--mount" ) {
            if (++i < argc)
                settings.mountPort = argv[i];
        }
        else if ( std::string(argv[i]) == "--lidar" ) {
            if (++i < argc)
                settings.lidarPort = argv[i];
        }
        else if ( std::string(argv[i]) == "--filename" ) {
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
        else if ( std::string(argv[i]) == "--voxel" ) {
            if (++i < argc)
                voxel = toFloat(std::string(argv[i])) * 0.01; // 1 cm = 0.01 meters
            else
                std::cout << "Argument '" << argument << "' should be followed by a the voxel size (expressed in cm) for the voxel grid. Default is" << voxel << std::endl;
        }
        else if ( std::string(argv[i]) == "--normals" )
            bNormal = true;
        else if ( std::string(argv[i]) == "--verbose" )
            bVerbose = true;
    }

    std::string extension = getExt(filename);
    if (extension.size() == 3) {
        formats.push_back(extension);
        filename = split(filename,'.',true)[0];
    }

    if (scanner.connect(settings, bVerbose)) {

        // Scan 75% degrees at half speed
        std::vector<glm::vec4> points = scanner.scan(toDegree, atSpeed, bVerbose);

        if (points.size() > 0) {

            // Declare Cloud data
            pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud( new pcl::PointCloud<pcl::PointXYZRGB> );
            cloud->width    = points.size();
            cloud->height   = 1;
            cloud->is_dense = true;
            cloud->points.resize (points.size());

            for (size_t i = 0; i < points.size(); i++) {
                cloud->points[i].x = points[i].x;
                cloud->points[i].y = points[i].y;
                cloud->points[i].z = points[i].z;
                unsigned char r, g, b;
                r = points[i].a * 255;
                g = points[i].a * 255;
                b = points[i].a * 255;
                // hue2rgb(points[i].a, r, g, b); 
                std::uint32_t rgb = packColor(r, g, b);
                cloud->points[i].rgb = *reinterpret_cast<float*>(&rgb);
            }

            if (voxel > 0.0) {
                pcl::VoxelGrid<pcl::PointXYZRGB> vg_filter;
                vg_filter.setInputCloud (cloud);
                vg_filter.setLeafSize (voxel, voxel, voxel);
                pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered( new pcl::PointCloud<pcl::PointXYZRGB> );
                vg_filter.filter(*cloud_filtered);
                *cloud = *cloud_filtered;
            }
            
            // if (bNormal) {
            //     std::cout << "Estimating normals" << std::endl;
            //     // // Normal estimation*
            //     pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> n;
            //     pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal>);
            //     pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
            //     tree->setInputCloud (cloud);
            //     n.setInputCloud (cloud);
            //     n.setSearchMethod (tree);
            //     n.setKSearch (10);
            //     n.compute (*normals);

            //     // Concatenate the XYZ and normal fields*
            //     pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals (new pcl::PointCloud<pcl::PointNormal>);
            //     pcl::concatenateFields (*cloud, *normals, *cloud_with_normals);

            //     savePointCloud(filename, formats, *cloud_with_normals);
            // }
            // else
                savePointCloud(filename, formats, *cloud);
        }

        scanner.reset(bVerbose);
        scanner.disconnect(false);
    }

    return 0;
}
