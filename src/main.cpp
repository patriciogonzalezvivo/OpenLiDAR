
#include <string>

#include "Scanner.h"

#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>

// Main program
//============================================================================
int main(int argc, char **argv){

    Scanner scanner;
    if (scanner.connect(argv[1], argv[2])) {
        std::cout << "Start Scanning" << std::endl;
        std::vector<glm::vec3> points = scanner.scan(SR_9);

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

            std::cout << "Saving points" << std::endl;
            pcl::io::savePLYFile ("point_cloud.ply", *cloud, false);
        }

        scanner.disconnect();
    }

    return 0;
}