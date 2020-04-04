#pragma once

#include <vector>

#include "drivers/mount/MountDriver.h"
#include "drivers/lidar/LidarDriver.h"
#include "drivers/gps/GpsDriver.h"

struct OpenLiDARSettings {
    MountType   mountType   = CELESTRON;
    char*       mountPort   = NULL;
    LidarType   lidarType   = RPLIDAR;
    char*       lidarPort   = NULL;
    GpsType     gpsType     = GPSD;
    char*       gpsPort     = (char*)"localhost";
};

class OpenLiDAR {
public:

    OpenLiDAR();
    virtual ~OpenLiDAR();

    bool        connect(OpenLiDARSettings& _settings, bool _verbose = true);
    bool        reset(bool _verbose = true);
    void        disconnect();

    bool        isScanning() { return m_scanning; }

    std::vector<glm::vec4> scan(float _toDegree, float _atSpeed, bool _verbose = true);
    
    GpsDriver*  getGps() { return m_gps; };

protected:
    MountDriver*    m_mount;
    LidarDriver*    m_lidar;
    GpsDriver*      m_gps;

    bool            m_scanning;
};