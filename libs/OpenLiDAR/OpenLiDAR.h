#pragma once

#include <vector>
#include <glm/glm.hpp>

#include "mount/Celestron.h"
#include "sensor/RPLidar.h"

class OpenLiDAR {
public:

    OpenLiDAR();
    virtual ~OpenLiDAR();

    bool    connect(const char* _celestronPort, const char* _rplidarPort);
    void    disconnect();

    bool    isScanning() { return m_scanning; }

    std::vector<glm::vec3> scan(CELESTRON_SLEW_RATE _rate);
    bool    reset();

protected:
    double          m_az;
    double          m_alt;

    Celestron*      m_mount;
    RPLidar*        m_lidar;

    bool            m_scanning;
};