#pragma once

#include "LidarDriver.h"
#include "rplidar.h"

#ifndef RPLIDAR_MAXSAMPLES
#define RPLIDAR_MAXSAMPLES 8192
#endif

using namespace rp::standalone::rplidar;

class RPLidar : public LidarDriver {
public:

    RPLidar();
    virtual ~RPLidar();

    bool        connect(const char* _portName, bool _verbose);
    void        disconnect();

    bool        printFirmware();

    bool        start(bool _verbose);
    bool        stop(bool _verbose);

    bool        getSamples(LidarSample* _samples, size_t& _count);

protected:
    rplidar_response_measurement_node_hq_t m_nodes[RPLIDAR_MAXSAMPLES];
    RPlidarDriver*  m_driver;
};