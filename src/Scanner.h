#pragma once

#include <vector>

#include "Celestron.h"
#include "rplidar.h"

#include <glm/glm.hpp>

class Scanner {
public:

    Scanner();
    virtual ~Scanner();

    bool    connect(const char* _celestronPort, const char* _rplidarPort);
    void    disconnect();

    bool    isScanning() { return m_scanning; }

    std::vector<glm::vec3> scan(CELESTRON_SLEW_RATE _rate);

protected:
    Celestron       m_mount;
    RPlidarDriver*  m_lidar;

    double          m_az;
    double          m_alt;

    bool            m_scanning;
};