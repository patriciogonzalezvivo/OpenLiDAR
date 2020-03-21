#pragma once

#include <vector>
#include <glm/glm.hpp>

#include "Celestron.h"
#include "rplidar.h"

using namespace rp::standalone::rplidar;

class Scanner {
public:

    Scanner();
    virtual ~Scanner();

    bool    connect(const char* _celestronPort, const char* _rplidarPort);
    void    disconnect();

    bool    isScanning() { return m_scanning; }

    std::vector<glm::vec3> scan(CELESTRON_SLEW_RATE _rate);
    bool    reset();

protected:
    double          m_az;
    double          m_alt;

    Celestron*      m_mount;
    RPlidarDriver*  m_lidar;

    bool            m_scanning;
};