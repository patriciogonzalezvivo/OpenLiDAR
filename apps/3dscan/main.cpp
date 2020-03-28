
#include <string>
#include <sstream>

#include "OpenLiDAR.h"

#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>

#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/features/normal_3d.h>

// glm::vec3 hsv2rgb(const glm::vec3& _hsb) {
//     glm::vec3 rgb = glm::clamp(   glm::abs(glm::mod(  glm::vec3(_hsb.x) * glm::vec3(6.) + glm::vec3(0., 4., 2.), 
//                                                     glm::vec3(6.)) - glm::vec3(3.) ) - glm::vec3(1.),
//                                 glm::vec3(0.),
//                                 glm::vec3(1.));
//     #ifdef HSV2RGB_SMOOTH
//     rgb = rgb*rgb*(3. - 2. * rgb);
//     #endif
//     return glm::vec3(_hsb.z) * glm::mix(glm::vec3(1.), rgb, _hsb.y);
// }

glm::vec3 hue(float _hue) {
    float R = abs(_hue * 6 - 3) - 1;
    float G = 2 - abs(_hue * 6 - 2);
    float B = 2 - abs(_hue * 6 - 4);
    return glm::clamp(glm::vec3(R,G,B), glm::vec3(0.0), glm::vec3(1.0));
}

uint32_t packRGB(uint8_t _r, uint8_t _g, uint8_t _b) {
    return ((uint32_t)_r << 16 | (uint32_t)_g << 8 | (uint32_t)_b);   
}

uint32_t packRGB(const glm::vec3& _color) {
    return packRGB(_color.r*255, _color.g*255, _color.b*255);
}

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

std::string getUniqueFileName( const std::string& _originalName) {
    std::string filename = _originalName + ".ply";
    int index = 0;
    while ( doFileExist( filename.c_str() ) ) {
        filename = _originalName + "_" + toString(index, 3, '0') + ".ply";
        index++;
    }
    return filename;
}


// Main program
//============================================================================
int main(int argc, char **argv){
    std::string portMount = "/dev/ttyUSB0";
    std::string portLidar = "/dev/ttyUSB1";
    std::string filename = "point_cloud";
    float loop = 0.9f;
    float speed = 0.75f;
    float leaf = 0.01f; // m
    bool bNormal = false;

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
        else if ( std::string(argv[i]) == "--loop" ) {
            if (++i < argc)
                loop = toFloat(std::string(argv[i]));
            else
                std::cout << "Argument '" << argument << "' should be followed by a the porcentage (expres in a number between 0.0 to 1.0) of a full turn. Default is " << loop << std::endl;
        }
        else if ( std::string(argv[i]) == "--speed" ) {
            if (++i < argc)
                speed = toFloat(std::string(argv[i]));
            else
                std::cout << "Argument '" << argument << "' should be followed by a the speed (expressed in a number between 0.0 to 1.0) of the speed of the scan. Default is " << speed << std::endl;
        }
        else if ( std::string(argv[i]) == "--leaf" ) {
            if (++i < argc)
                leaf = toFloat(std::string(argv[i]));
            else
                std::cout << "Argument '" << argument << "' should be followed by a the leaf size (expressed in meters) for the voxel grid. Default is" << leaf << std::endl;
        }
        else if ( std::string(argv[i]) == "--normals" )
            bNormal = true;
    }

    OpenLiDAR scanner;
    if (scanner.connect(portMount.c_str(), portLidar.c_str())) {
        std::cout << "Start Scanning" << std::endl;

        // Scan 75% loop at half speed
        std::vector<glm::vec4> points = scanner.scan(loop, speed);
        float time_end = points[points.size()-1].w;

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
                cloud->points[i].rgb = packRGB(hue(points[i].w/time_end));
            }

            if (leaf > 0.0) {
                pcl::VoxelGrid<pcl::PointXYZRGB> vg_filter;
                vg_filter.setInputCloud (cloud);
                vg_filter.setLeafSize (leaf, leaf, leaf);
                pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered( new pcl::PointCloud<pcl::PointXYZRGB> );
                vg_filter.filter(*cloud_filtered);
                *cloud = *cloud_filtered;
            }
            
            filename = getUniqueFileName(filename);

            if (bNormal) {
                std::cout << "Estimating normals" << std::endl;
                // // Normal estimation*
                pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> n;
                pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal>);
                pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB>);
                tree->setInputCloud (cloud);
                n.setInputCloud (cloud);
                n.setSearchMethod (tree);
                n.setKSearch (10);
                n.compute (*normals);

                // Concatenate the XYZ and normal fields*
                pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_with_normals (new pcl::PointCloud<pcl::PointXYZRGBNormal>);
                pcl::concatenateFields (*cloud, *normals, *cloud_with_normals);

                std::cout << "Saving points on " << filename << std::endl;
                pcl::io::savePLYFile(filename, *cloud_with_normals, false);
            }
            else {
                std::cout << "Saving points on " << filename << std::endl;
                pcl::io::savePLYFile(filename, *cloud, false);
            }
        }

        std::cout << "Reset scanner" << std::endl;
        scanner.reset();
        // scanner.disconnect();
    }

    return 0;
}
