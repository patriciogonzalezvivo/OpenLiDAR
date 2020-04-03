#pragma once

#include <vector>

#include "mount/MountDriver.h"
#include "lidar/LidarDriver.h"
#include "gps/GpsDriver.h"

class OpenLiDAR {
public:

    OpenLiDAR();
    virtual ~OpenLiDAR();

    bool        connect(LidarType _lidarType, MountType _mountType, bool _verbose);
    bool        connect(const char* _lidarPort, const char* _mountPort, bool _verbose);
    void        disconnect();

    bool        isScanning() { return m_scanning; }

    std::vector<glm::vec4> scan(float _toDegree, float _atSpeed, bool _verbose);
    bool        reset(bool _verbose = true);

protected:
    MountDriver*    m_mount;
    LidarDriver*    m_lidar;
    GpsDriver*      m_gps;

    bool            m_scanning;
};