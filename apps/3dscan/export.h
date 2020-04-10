#pragma once

#include <pcl/point_types.h>

#include <pcl/io/ply_io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/png_io.h>
#include <pcl/range_image/range_image_spherical.h>
#include <pcl/visualization/common/float_image_utils.h>

#include "tools/colorOps.h"



template<typename PointT> 
inline bool savePointCloud( const std::string& _filename, 
                            const std::vector<std::string>& _formats, 
                            const pcl::PointCloud<PointT> &_cloud) 
{
    for (size_t i = 0; i < _formats.size(); i++) {
        std::string filename = getUniqueFileName(_filename, _formats[i]);
        std::cout << "Saving points as " << filename << std::endl;
 
        if (_formats[i] == "ply") {
            pcl::PLYWriter w;
            w.write<PointT> (filename, _cloud, false, false);
        }

        else if (_formats[i] == "pcd")
            pcl::io::savePCDFile(filename, _cloud, true);
 
        else if (_formats[i] == "png") {
            // We now want to create a range image from the above point cloud, with a 1deg angular resolution
            float angularResolution = glm::radians(  0.5f);  //   1.0 degree in radians
            float maxAngleWidth     = glm::radians(360.0f);  // 360.0 degree in radians
            float maxAngleHeight    = glm::radians(180.0f);  // 180.0 degree in radians
            Eigen::Affine3f sensorPose = (Eigen::Affine3f)Eigen::Translation3f(0.0f, 0.0f, 0.0f);
            Eigen::Vector3f point_cloud_center = Eigen::Vector3f(0.0,0.0,0.0);
            float point_cloud_radius = 2.0;
            pcl::RangeImage::CoordinateFrame coordinate_frame = pcl::RangeImage::CAMERA_FRAME;
            float noiseLevel = 0.0f;
            float minRange = 0.0f;
            int borderSize = 0;
            
            pcl::RangeImageSpherical rangeImage;
            rangeImage.createFromPointCloud(_cloud, angularResolution, maxAngleWidth, maxAngleHeight,
                                            sensorPose, coordinate_frame, noiseLevel, minRange, borderSize);

            std::cout << "Saving points as " << filename << std::endl;
            float* ranges = rangeImage.getRangesArray();
            // unsigned char* rgb_image = pcl::visualization::FloatImageUtils::getVisualImage (ranges, rangeImage.width, rangeImage.height); 
            unsigned char* rgb_image = toRGB(ranges, rangeImage.width, rangeImage.height); 
            pcl::io::saveRgbPNGFile(filename.c_str(), rgb_image, rangeImage.width, rangeImage.height); 
        }
    }

    return true;
}