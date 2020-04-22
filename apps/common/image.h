#pragma once

#include <string>

class Image {
public:

    Image();
    Image(const std::string& _filename, int _channels);
    virtual ~Image();

    bool        allocate(int _width, int _height, int _channels);
    bool        load(const std::string& _filename, int _channels);
    bool        save(const std::string& _filename);

    static bool savePNG(const std::string& _path, unsigned char* _pixels, int _width, int _height, int _channels);
    static bool savePNG16(const std::string& _path, uint16_t* _pixels, int _width, int _height, int _channels);
    static bool saveHDR(const std::string& _path, float* _pixels, int _width, int _height, int _channels);

    int         getWidth() {return m_width;}
    int         getHeight() {return m_height;};
    int         getChannels() {return m_channels;};
    size_t      getIndex(int _x, int _y) { return (_y * m_width + _x) * m_channels; };
    size_t      getIndexUV(float _u, float _v) { return getIndex(_u * m_width, _v * m_height); }

    uint32_t    getData(int _x, int _y);
    uint32_t    getDataUV(float _u, float _v) { return getData(int(_u * m_width), int(_v * m_height)); }

    void        setData(int _index, uint16_t* _data, int _size);
    void        setData(int _x, int _y, uint16_t* _data, int _size) { setData(_x, _y, _data, _size); }
        
private:
    uint16_t*   m_data;
    int         m_width;
    int         m_height;
    int         m_channels;

};