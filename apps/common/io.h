#pragma once

#include <pcl/point_types.h>

#include <pcl/io/ply_io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/png_io.h>
#include <pcl/range_image/range_image_spherical.h>
#include <pcl/visualization/common/float_image_utils.h>

#include "mary/tools/textOps.h"
#include "fileOps.h"
#include "colorOps.h"

template<typename PointT> 
inline bool loadPointCloud( const std::string& _filename,
                            pcl::PointCloud<PointT> &_cloud) 
{
    std::string ext = getExt(_filename);
    if (ext == "pcd") {
        if (pcl::io::loadPCDFile<PointT> (_filename.c_str(), _cloud) == -1) {
            PCL_ERROR ("Couldn't read file &s\n", _filename);
            return false;
        }
    }
    else if (ext == "ply") {
        if (pcl::io::loadPLYFile<PointT> (_filename.c_str(), _cloud) == -1) {
            PCL_ERROR ("Couldn't read file &s\n", _filename);
            return false;
        }
    }

    return true;
}

template<typename T>
inline void flipPixelsVertically(T *_pixels, int _width, int _height, int _bytes_per_pixel) {
    const size_t stride = _width * _bytes_per_pixel;
    T *row = (T*)malloc(stride * sizeof(T));
    T *low = _pixels;
    T *high = &_pixels[(_height - 1) * stride];
    for (; low < high; low += stride, high -= stride) {
        std::memcpy(row, low, stride * sizeof(T));
        std::memcpy(low, high, stride * sizeof(T));
        std::memcpy(high, row, stride * sizeof(T));
    }
    free(row);
}

// http://stackoverflow.com/questions/466204/rounding-up-to-nearest-power-of-2
inline int nextPowerOfTwo (float x) {
    return (int)pow(2.0f, ceil(log(x) / log(2.0f)));
}

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

            float* ranges = rangeImage.getRangesArray();
            // unsigned char* rgb_image = pcl::visualization::FloatImageUtils::getVisualImage (ranges, rangeImage.width, rangeImage.height); 
            unsigned char* rgb_image = toRGB(ranges, rangeImage.width, rangeImage.height); 
            flipPixelsVertically<unsigned char>(rgb_image, rangeImage.width, rangeImage.height, 3);
            pcl::io::saveRgbPNGFile(filename.c_str(), rgb_image, rangeImage.width, rangeImage.height); 
        }
        else if (_formats[i] == "PNG") {
            int totalPoints = _cloud.points.size();
            int textureSize = nextPowerOfTwo( sqrt(totalPoints) );
            // NORMALIZE 
            size_t totalPixels = textureSize*textureSize;
            double min_x = 100000.0;    double max_x = -100000.0;
            double min_y = 100000.0;    double max_y = -100000.0;
            double min_z = 100000.0;    double max_z = -100000.0;
            for (size_t i = 0; i < _cloud.points.size(); i++) {
                // X
                if (_cloud.points[i].x < min_x)
                    min_x = _cloud.points[i].x;
                else if (_cloud.points[i].x > max_x)
                    max_x = _cloud.points[i].x;

                // Y
                if (_cloud.points[i].y < min_y)
                    min_y = _cloud.points[i].y;
                else if (_cloud.points[i].y > max_y)
                    max_y = _cloud.points[i].y;

                // Z
                if (_cloud.points[i].z < min_z)
                    min_z = _cloud.points[i].z;
                else if (_cloud.points[i].z > max_z)
                    max_z = _cloud.points[i].z;
            }
            std::cout << "bound_min," << min_x << "," << min_y << "," << min_z << std::endl;
            std::cout << "bound_max," << max_x << "," << max_y << "," << max_z << std::endl;
 
            unsigned short*  pixels_xyz = new unsigned short[totalPixels * 3];
            std::fill_n(pixels_xyz, totalPixels * 3, 0);

            // unsigned char*  pixels_rgb = new unsigned char[totalPixels * 4];
            // std::fill_n(pixels_rgb, totalPixels * 4, 0);

            for (size_t i = 0; i < totalPoints; i++) {
                pixels_xyz[i * 3 + 0] = map(_cloud.points[i].x, min_x, max_x, 0, 65535);
                pixels_xyz[i * 3 + 1] = map(_cloud.points[i].y, min_y, max_y, 0, 65535);
                pixels_xyz[i * 3 + 2] = map(_cloud.points[i].z, min_z, max_z, 0, 65535);

                // pixels_rgb[i * 4 + 0] = _cloud.points[i].r;
                // pixels_rgb[i * 4 + 1] = _cloud.points[i].g;
                // pixels_rgb[i * 4 + 2] = _cloud.points[i].b;
                // pixels_rgb[i * 4 + 3] = _cloud.points[i].a;
            }

            flipPixelsVertically<unsigned short>(pixels_xyz, textureSize, textureSize, 3);
            pcl::io::saveShortPNGFile (filename.c_str(), pixels_xyz, textureSize, textureSize, 3);
            delete pixels_xyz;

            // flipPixelsVertically<unsigned short>(pixels_xyz, textureSize, textureSize, 4);
            // Image::saveCharPNGFile(filename.c_str(), pixels_rgb, textureSize, textureSize, 4);
            // delete pixels_rgb;
        }
    }

    return true;
}