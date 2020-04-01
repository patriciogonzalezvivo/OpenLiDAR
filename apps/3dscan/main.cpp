
#include <string>
#include <sstream>

#include "OpenLiDAR.h"

#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>

#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/features/normal_3d.h>

float toFloat(const std::string& _string) {
    float x = 0;
    std::istringstream cur(_string);
    cur >> x;
    return x;
}

/// like sprintf "% 4d" or "% 4f" format, in this example width=4, fill=' '
template <class T>
inline std::string toString(const T& _value, int _width, char _fill) {
    std::ostringstream out;
    out << std::fixed << std::setfill(_fill) << std::setw(_width) << _value;
    return out.str();
}

bool doFileExist(const char *_fileName) {
    std::ifstream infile(_fileName);
    return infile.good();
}

std::string getUniqueFileName( const std::string& _originalName, const std::string& _extension) {
    std::string filename = _originalName + "." + _extension;
    int index = 0;
    while ( doFileExist( filename.c_str() ) ) {
        filename = _originalName + "_" + toString(index, 3, '0') + "." + _extension;
        index++;
    }
    return filename;
}


// Main program
//============================================================================
int main(int argc, char **argv){
    OpenLiDAR scanner;

    std::string portLidar = "/dev/ttyUSB0";
    std::string portMount = "/dev/ttyUSB1";
    std::string filename = "point_cloud";
    float toDegree = 180.0f; // Half loop
    float atSpeed = 0.75f;
    float leaf = 0.01f;     // m
    bool bNormal = false;
    bool bVerbose = false;


    for (int i = 1; i < argc ; i++) {
        std::string argument = std::string(argv[i]);

        if ( std::string(argv[i]) == "--mount" ) {
            if (++i < argc)
                portMount = std::string(argv[i]);
            else
                std::cout << "Argument '" << argument << "' should be followed by the serial adrees of the mount device. Default is " << portMount << std::endl;
        }
        else if ( std::string(argv[i]) == "--lidar" ) {
            if (++i < argc)
                portLidar = std::string(argv[i]);
            else
                std::cout << "Argument '" << argument << "' should be followed by the serial adrees of the lidar device. Default is " << portLidar << std::endl;
        }
        else if ( std::string(argv[i]) == "--out" ) {
            if (++i < argc)
                filename = std::string(argv[i]);
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

    
    if (scanner.connect(portLidar.c_str(), portMount.c_str(), bVerbose)) {

        // Scan 75% degrees at half speed
        std::vector<glm::vec4> points = scanner.scan(toDegree, atSpeed, bVerbose);
        float time_end = points[points.size()-1].w;

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
            
            filename = getUniqueFileName(filename, "ply");

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

                std::cout << "Saving points on " << filename << std::endl;
                pcl::io::savePLYFile(filename, *cloud_with_normals, false);
            }
            else {
                std::cout << "Saving points on " << filename << std::endl;
                pcl::io::savePLYFile(filename, *cloud, false);
            }
        }
        
        scanner.reset(bVerbose);
        scanner.disconnect();
    }

    return 0;
}
