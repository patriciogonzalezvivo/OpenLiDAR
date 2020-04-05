#include "OpenLiDAR.h"

#include <unistd.h>
#include <iostream>

#define GLM_ENABLE_EXPERIMENTAL
#include "glm/gtx/transform.hpp"
#include "glm/gtc/quaternion.hpp"
#include "glm/gtx/quaternion.hpp"

#include "tools/timeOps.h"
#include "tools/textOps.h"

#include "drivers/gps/Gpsd.h"
#include "drivers/lidar/RPLidar.h"
#include "drivers/mount/Celestron.h"

OpenLiDAR::OpenLiDAR() :
    m_mount(NULL),
    m_lidar(NULL),
    m_gps(NULL),
    m_scanning(false) {
}

OpenLiDAR::~OpenLiDAR(){
    disconnect();
}

bool OpenLiDAR::initDrivers(OpenLiDARSettings& _settings, bool _verbose) {
    // Initialize drivers
    if (!m_mount) {
        switch (_settings.mountType) {
        case CELESTRON:
            m_mount = new Celestron();
            break;
        
        default:
            break;
        }
    }

    if (!m_lidar) {
        switch (_settings.lidarType) {
        case RPLIDAR:
            m_lidar = new RPLidar();
            break;
        
        default:
            break;
        }
    }


    if (!m_gps) {
        switch (_settings.gpsType) {
        case GPSD:
            m_gps = new Gpsd();
            break;
        
        default:
            break;
        }
    }

    return true;
}

bool OpenLiDAR::fillPortDrivers(OpenLiDARSettings& _settings, bool _verbose) {
    initDrivers(_settings, _verbose);

        // GET DRIVERS PORTS if there are not
    if (!_settings.mountPort)
        _settings.mountPort = m_mount->getPort();
    
    if (!_settings.lidarPort)
        _settings.lidarPort = m_lidar->getPort();

    if (_verbose) {
        std::cout << "Loading Mount from " << _settings.mountPort << std::endl;
        std::cout << "Loading LiDAR from " << _settings.lidarPort << std::endl;
        std::cout << "Loading GPS from " << _settings.gpsPort << std::endl;
    }

    disconnect();

    return true;
}

bool OpenLiDAR::connect(OpenLiDARSettings& _settings, bool _verbose) {
    if (_settings.mountPort == NULL || _settings.lidarPort == NULL)
        fillPortDrivers(_settings, _verbose);

    initDrivers(_settings, _verbose);

    // CONNECT DRIVERS
    if (!m_mount->connect(_settings.mountPort, _verbose)) {
        std::cerr << "Can't load Mount from " << _settings.mountPort << std::endl;
        delete m_mount;
        m_mount = NULL;
    }

    if (!m_lidar->connect(_settings.lidarPort, _verbose)) {
        std::cerr << "Can't load LiDAR Sensor from " << _settings.lidarPort << std::endl;
        delete m_lidar;
        m_lidar = NULL;
    }

    if (!m_gps->connect(_settings.gpsPort, _verbose)) {
        std::cerr << "Can't load GPS from localhost" << std::endl;
        delete m_gps;
        m_gps = NULL;
    }

#if defined(DEBUG_USING_SIMULATE_DATA)
    if (m_lidar == NULL || m_mount == NULL)
        std::cout << "WARNING!!! Basic devices are not connected, data will be simulated." << std::endl;

    return true;
#endif

    return (m_lidar != NULL) && (m_mount != NULL);
}

void OpenLiDAR::disconnect(bool _verbose) {
    if (m_mount) {
        if (_verbose)
            std::cout << "Disconnecting MOUNT driver" << std::endl;
        m_mount->disconnect();
        delete m_mount;
        m_mount = NULL;
    }

    if (m_lidar) {
        if (_verbose)
            std::cout << "Disconnecting LiDAR driver" << std::endl;
        m_lidar->disconnect();
        delete m_lidar;
        m_lidar = NULL;
    }

    if (m_gps) {
        if (_verbose)
            std::cout << "Disconnecting GPS driver" << std::endl;
        m_gps->disconnect();
        delete m_gps;
        m_gps = NULL;
    }
}

std::vector<glm::vec4> OpenLiDAR::scan(float _toDegree, float _atSpeed, bool _verbose) {
    if (_verbose) 
        std::cout << "Start Scanning..." << std::endl;

    m_scanning = true;
    glm::vec3 offset = glm::vec3(0.0,0.0,0.0);
    double az = 0.0;
    double start_time = getElapsedSeconds();
    std::vector<glm::vec4> points;

    // Start GPSScanning
    // if (m_gps)
    //     m_gps->start(_verbose);

    // Start motor...
    if (m_mount) {
        m_mount->start(_atSpeed, _verbose);
        az = m_mount->getAz();
        offset += m_mount->getOffset();
    }

    // Start sensor...
    if (m_lidar) {
        m_lidar->start(_verbose);
        offset.x += m_lidar->getHeight();
    }

    // Initialize needed variables to gather data
    size_t count = 0;
    LidarSample samples[RPLIDAR_MAXSAMPLES];
    // fetch result and print it out...
    while (m_scanning && az < _toDegree) {
        float delta_time = float(getElapsedSeconds() - start_time);

        // Get mount azimuth angle
        if (m_mount) 
            az = m_mount->getAz();
        #if defined(DEBUG_USING_SIMULATE_DATA)
        // SIMULATE DATA
        else 
            az += 0.5;
        #endif

        glm::quat lng = glm::angleAxis(float(glm::radians(-az)), glm::vec3(0.0,1.0,0.0));

        if (m_lidar) {

            if (!m_lidar->getSamples(samples, count))
                continue;

            for (size_t i = 0; i < count ; i++) {
                glm::quat lat = glm::angleAxis(glm::radians(-samples[i].theta), glm::vec3(1.0,0.0,0.0));
                glm::vec3 pos = lng * (lat * glm::vec3(0.0, 0.0, samples[i].distance) + offset);
                points.push_back( glm::vec4(pos, delta_time) );
            }
        }
        #if defined(DEBUG_USING_SIMULATE_DATA)
        else {
            // SIMULATE DATA
            for (int i = 0; i < 8000 ; i++) {
                glm::quat lat = glm::angleAxis(glm::radians(i * 0.045f), glm::vec3(1.0,0.0,0.0));
                glm::vec3 pos = lng * (lat * glm::vec3(0.0, 0.0, 1.0) + offset);
                points.push_back( glm::vec4(pos, delta_time) );
            }
        }
        #endif

        if (_verbose) {
            // Delete previous line
            const std::string deleteLine = "\e[2K\r\e[1A";
            std::cout << deleteLine;

            int pct = (az/_toDegree) * 100;
            
            std::cout << " [ ";
            for (int i = 0; i < 50; i++) {
                if (i < pct/2) std::cout << "#";
                else std::cout << ".";
            }
            std::cout << " ] " << toMMSS(delta_time) << " az: " << toString(az,1,3,'0') << " pts: " << points.size() << std::endl;
        }
    }

    // Stop capturing Lidar data
    if (m_lidar)
        m_lidar->stop(_verbose);

    // Stop panning 
    if (m_mount)
        m_mount->stop(_verbose);

    // Stop GPS
    // if (m_gps)
    //     m_gps->stop(_verbose);

    m_scanning = false;

    return points;
}

bool OpenLiDAR::reset(bool _verbose) {
    if (m_mount)
        return m_mount->reset(_verbose);
    
    #if defined(DEBUG_USING_SIMULATE_DATA)
    return true;
    #endif
    return false;
}
