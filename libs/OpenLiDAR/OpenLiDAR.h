#pragma once

#include <vector>
#include <glm/glm.hpp>

#include "mount/Celestron.h"
#include "sensor/RPLidar.h"

class OpenLiDAR {
public:

    OpenLiDAR();
    virtual ~OpenLiDAR();

    bool        connect(const char* _celestronPort, const char* _rplidarPort);
    void        disconnect();

    bool        isScanning() { return m_scanning; }

    void        setOffsetX(float _x) { m_offset.x = _x; }
    void        setOffsetY(float _y) { m_offset.x = _y; }
    void        setOffsetZ(float _z) { m_offset.x = -_z; }
    glm::vec3   getOffset() { return m_offset; }

    std::vector<glm::vec4> scan(float _degree = 180.0, float _speed = 0.5, bool _verbose = true);
    bool        reset(bool _verbose = true);

protected:
    glm::vec3       m_offset;
    double          m_az;
    double          m_alt;

    Celestron*      m_mount;
    RPLidar*        m_lidar;

    bool            m_scanning;
};