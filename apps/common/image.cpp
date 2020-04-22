#include <iostream>
#include <stdio.h>

#include <png.h>

#include "image.h"

#define STB_IMAGE_IMPLEMENTATION
#include "stb_image.h"

#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "stb_image_write.h"


Image::Image(): m_width(0), m_height(0), m_channels(0), m_data(nullptr) {
}

Image::Image(const std::string& _filename, int _channels) {
    load(_filename, _channels);
}

Image::~Image() {
    if (m_data)
        delete [] m_data;
}

bool Image::allocate(int _width, int _height, int _channels) {
    m_width = _width;
    m_height = _height;
    m_channels = _channels;
    m_data = new uint16_t[_width * _height * _channels];
    memset(&m_data[0], 0, sizeof(m_data));
    return true;
}
   
bool Image::load(const std::string& _filename, int _channels) {
    m_channels = _channels;
    // stbi_set_flip_vertically_on_load(true);
    int comp;
    m_data = stbi_load_16(_filename.c_str(), &m_width, &m_height, &comp, m_channels);
    return true;
}

bool hasEnding (std::string const &fullString, std::string const &ending) {
    if (fullString.length() >= ending.length()) {
        return (0 == fullString.compare (fullString.length() - ending.length(), ending.length(), ending));
    } else {
        return false;
    }
}

bool Image::savePNG(const std::string& _filename, unsigned char* _pixels, int _width, int _height, int _channels) {
    if (0 == stbi_write_png(_filename.c_str(), _width, _height, _channels, _pixels, 0)) {
        std::cout << "Can't create file " << _filename << std::endl;
        return false;
    }
    return true;
}

bool Image::savePNG16(const std::string& _filename, uint16_t* _pixels, int _width, int _height, int _channels){

    /* create file */
    FILE *fp = fopen(_filename.c_str(), "wb");
    if (!fp){
        std::runtime_error("[write_png_file] File could not be opened for writing");
        return false;
    }


    /* initialize stuff */
    png_structp png_ptr = png_create_write_struct(PNG_LIBPNG_VER_STRING, NULL, NULL, NULL);

    if (!png_ptr){
        std::runtime_error("[write_png_file] png_create_write_struct failed");
        return false;
    }

    png_infop info_ptr = png_create_info_struct(png_ptr);
    if (!info_ptr){
        std::runtime_error("[write_png_file] png_create_info_struct failed");
        return false;
    }

    if (setjmp(png_jmpbuf(png_ptr))){
        std::runtime_error("[write_png_file] Error during init_io");
        return false;
    }

    png_init_io(png_ptr, fp);


    /* write header */
    if (setjmp(png_jmpbuf(png_ptr))){
        std::runtime_error("[write_png_file] Error during writing header");
        return false;
    }

    png_set_IHDR(png_ptr, info_ptr,
            (png_uint_32) _width, (png_uint_32) _height,
            (png_byte) 16, PNG_COLOR_TYPE_RGB, PNG_INTERLACE_NONE,
            PNG_COMPRESSION_TYPE_DEFAULT, PNG_FILTER_TYPE_DEFAULT);

    png_write_info(png_ptr, info_ptr);


    /* write bytes */
    if (setjmp(png_jmpbuf(png_ptr))){
        std::runtime_error("[write_png_file] Error during writing bytes");
        return false;
    }

    /* Initialize rows of PNG. */
    png_bytepp row_pointers = (png_bytepp) malloc (sizeof(png_bytep)*_height);
    png_bytep row = (png_bytep) malloc(6 * _width * sizeof(png_byte));
    for (size_t y = 0; y < _height; ++y) {
        row_pointers[y] = (png_bytep) malloc (png_get_rowbytes(png_ptr,info_ptr));
        for (size_t x = 0; x < _width; ++x) {
            size_t i = y * _width + x;
            png_save_uint_16(&row[x*6+0], (unsigned int)_pixels[i * _channels + 0] );
            png_save_uint_16(&row[x*6+2], (unsigned int)_pixels[i * _channels + 1] );
            png_save_uint_16(&row[x*6+4], (unsigned int)_pixels[i * _channels + 2] );
        }
        png_write_row(png_ptr, row);
    }

    /* end write */
    if (setjmp(png_jmpbuf(png_ptr))){
        std::runtime_error("[write_png_file] Error during end of write");
        return false;
    }

    png_write_end(png_ptr, NULL);
    fclose(fp);
    free(row);

    return true;
}


bool Image::saveHDR(const std::string& _filename, float* _pixels, int _width, int _height, int _channels) {
    if (0 == stbi_write_hdr(_filename.c_str(), _width, _height, _channels, _pixels)) {
        std::cout << "Can't create file " << _filename << std::endl;
        return false;
    }
    return true;
}

bool Image::save(const std::string& _filename) {
    return savePNG16(_filename, m_data, m_width, m_height, m_channels);
}

std::uint32_t Image::getData(int _x, int _y) {
    if (m_data == nullptr)
        return 0;

    int index = getIndex(_x, _y);

    if (m_channels == 3) {
        uint8_t r = m_data[index];
        uint8_t g = m_data[index + 1];
        uint8_t b = m_data[index + 2];
        return ((uint32_t)r << 16 | (uint32_t)g << 8 | (uint32_t)b);    
    }
    else if (m_channels == 1){
        return (uint32_t)m_data[index];
    }
    else {
        return 0;
    }
}

void Image::setData(int _index, uint16_t* _data, int _size) {
    if (m_data == nullptr) {
        std::cout << "Data have been not pre allocated" << std::endl;
        return;
    }

    if (_size != m_channels) {
        std::cout << "The size of the data doesn't match the pre allocated space for it" << std::endl;
        return;
    }

    for (size_t i = 0; i < _size; i++)
        m_data[_index + i] = _data[i];

}