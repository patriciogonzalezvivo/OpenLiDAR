#pragma once

#include <vector>

#include "mount/Celestron.h"
#include "lidar/RPLidar.h"
#include <libgpsmm.h>

class OpenLiDAR {
public:

    OpenLiDAR();
    virtual ~OpenLiDAR();

    bool        connect(bool _verbose);
    bool        connect(const char* _lidarPort, const char* _mountPort, bool _verbose);
    void        disconnect();

    bool        isScanning() { return m_scanning; }

    std::vector<glm::vec4> scan(float _toDegree, float _atSpeed, bool _verbose);
    bool        reset(bool _verbose = true);

protected:
    Mount*          m_mount;
    Lidar*          m_lidar;

    bool            m_scanning;
};