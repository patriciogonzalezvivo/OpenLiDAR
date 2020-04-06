
#include "OpenLiDAR.h"
#include "tools/textOps.h"
#include "tools/fileOps.h"

#include <pcl/point_types.h>

#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/features/normal_3d.h>

#include <pcl/io/ply_io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/png_io.h>
#include <pcl/range_image/range_image.h>
#include <pcl/visualization/common/float_image_utils.h>

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
        else if (_formats[i] == "png") {
            // We now want to create a range image from the above point cloud, with a 1deg angular resolution
            float angularResolution = glm::radians(  0.5f);  //   1.0 degree in radians
            float maxAngleWidth     = glm::radians(360.0f);  // 360.0 degree in radians
            float maxAngleHeight    = glm::radians(180.0f);  // 180.0 degree in radians
            Eigen::Affine3f sensorPose = (Eigen::Affine3f)Eigen::Translation3f(0.0f, 0.0f, 0.0f);
            Eigen::Vector3f point_cloud_center = Eigen::Vector3f(0.0,0.0,0.0);
            float point_cloud_radius = 2.0;
            pcl::RangeImage::CoordinateFrame coordinate_frame = pcl::RangeImage::LASER_FRAME;
            float noiseLevel = 0.0f;
            float minRange = 0.0f;
            int borderSize = 0;
            
            pcl::RangeImage rangeImage;
            rangeImage.createFromPointCloud(_cloud, angularResolution, maxAngleWidth, maxAngleHeight,
                                            sensorPose, coordinate_frame, noiseLevel, minRange, borderSize);
            // rangeImage.createFromPointCloudWithKnownSize(   _cloud, angularResolution, 
            //                                                 point_cloud_center, point_cloud_radius,
            //                                                 sensorPose, coordinate_frame, noiseLevel, minRange, borderSize);

            std::cout << "Saving points as " << filename << std::endl;
            float* ranges = rangeImage.getRangesArray();
            unsigned char* rgb_image = pcl::visualization::FloatImageUtils::getVisualImage (ranges, rangeImage.width, rangeImage.height); 
            pcl::io::saveRgbPNGFile(filename.c_str(), rgb_image, rangeImage.width, rangeImage.height); 
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

            if (voxel > 0.0) {
                pcl::VoxelGrid<pcl::PointXYZ> vg_filter;
                vg_filter.setInputCloud (cloud);
                vg_filter.setLeafSize (voxel, voxel, voxel);
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
        scanner.disconnect(false);
    }

    return 0;
}
