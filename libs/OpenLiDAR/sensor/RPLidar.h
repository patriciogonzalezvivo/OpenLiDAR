#pragma once

#include "../Driver.h"
#include "rplidar.h"

#ifndef RPLIDAR_MAXSAMPLES
#define RPLIDAR_MAXSAMPLES 8192
#endif

struct LidarSample {
    float theta;
    float distance;
};

using namespace rp::standalone::rplidar;

class RPLidar : public Driver {
public:

    RPLidar();
    virtual ~RPLidar();

    bool        connect(const char* _portName);
    void        disconnect();

    bool        printFirmware();

    bool        start();
    bool        stop();

    bool        getSamples(LidarSample* _samples, size_t& _count);

protected:
    rplidar_response_measurement_node_hq_t m_nodes[RPLIDAR_MAXSAMPLES];
    RPlidarDriver*  m_driver;
};