#pragma once

#include <vector>

#include "drivers/mount/MountDriver.h"
#include "drivers/lidar/LidarDriver.h"
// #include "drivers/gps/GpsDriver.h"

struct OpenLiDARSettings {
    MountType   mountType   = CELESTRON;
    char*       mountPort   = NULL;
    LidarType   lidarType   = RPLIDAR;
    char*       lidarPort   = NULL;
    // GpsType     gpsType     = GPSD;
    // char*       gpsPort     = (char*)"localhost";
};

class OpenLiDAR {
public:

    OpenLiDAR();
    virtual ~OpenLiDAR();

    bool        connect(OpenLiDARSettings& _settings, bool _verbose = true);
    bool        reset(bool _verbose = true);
    void        disconnect(bool _verbose = true);

    std::vector<glm::vec4> scan(float _toDegree, float _atSpeed, bool _verbose = true);
    
    MountDriver*    getMount() { return m_mount; };
    LidarDriver*    getLidar() { return m_lidar; };

protected:
    bool            initDrivers(OpenLiDARSettings& _settings, bool _verbose);
    bool            fillPortDrivers(OpenLiDARSettings& _settings, bool _verbose);

    MountDriver*    m_mount;
    LidarDriver*    m_lidar;
};