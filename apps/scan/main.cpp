
#include <string>
#include <sstream>

#include "OpenLiDAR.h"

#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>

#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/features/normal_3d.h>

const float LEAF_SIZE = 0.01f; // m

glm::vec3 hsv2rgb(const glm::vec3& _hsb) {
    glm::vec3 rgb = glm::clamp(   glm::abs(glm::mod(  glm::vec3(_hsb.x) * glm::vec3(6.) + glm::vec3(0., 4., 2.), 
                                                    glm::vec3(6.)) - glm::vec3(3.) ) - glm::vec3(1.),
                                glm::vec3(0.),
                                glm::vec3(1.));
    #ifdef HSV2RGB_SMOOTH
    rgb = rgb*rgb*(3. - 2. * rgb);
    #endif
    return glm::vec3(_hsb.z) * glm::mix(glm::vec3(1.), rgb, _hsb.y);
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


// Main program
//============================================================================
int main(int argc, char **argv){
    float loop = 0.9f;
    float speed = 0.75f;

    if (argc > 1) loop = toFloat(std::string(argv[1]));
    if (argc > 2) speed = toFloat(std::string(argv[2]));

    OpenLiDAR scanner;
    if (scanner.connect()) {
        std::cout << "Start Scanning" << std::endl;

        // Scan 75% loop at half speed
        std::vector<glm::vec4> points = scanner.scan(loop, speed);
        float time_end = points[points.size()-1].w;

        std::cout << "Got " << points.size() << " points" << std::endl;
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
                cloud->points[i].rgb = packRGB(hsv2rgb(glm::vec3(points[i].w/time_end, 1., 1.)));
            }
            
            pcl::VoxelGrid<pcl::PointXYZRGB> vg_filter;
            vg_filter.setInputCloud (cloud);
            vg_filter.setLeafSize (LEAF_SIZE, LEAF_SIZE, LEAF_SIZE);
            pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered( new pcl::PointCloud<pcl::PointXYZRGB> );
            vg_filter.filter(*cloud_filtered);

            // // Normal estimation*
            pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> n;
            pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal>);
            pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB>);
            tree->setInputCloud (cloud_filtered);
            n.setInputCloud (cloud_filtered);
            n.setSearchMethod (tree);
            n.setKSearch (10);
            n.compute (*normals);

            // Concatenate the XYZ and normal fields*
            pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_with_normals (new pcl::PointCloud<pcl::PointXYZRGBNormal>);
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
