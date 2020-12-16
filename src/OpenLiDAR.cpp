#include "OpenLiDAR.h"

#include <unistd.h>
#include <iostream>

#define GLM_ENABLE_EXPERIMENTAL
#include "glm/gtx/transform.hpp"
#include "glm/gtc/quaternion.hpp"
#include "glm/gtx/quaternion.hpp"

#include "mary/tools/timeOps.h"
#include "mary/tools/textOps.h"

// #include "drivers/gps/Gpsd.h"
#include "mary/drivers/lidar/RPLidar.h"
#include "mary/drivers/mount/Celestron.h"

OpenLiDAR::OpenLiDAR() :
    m_mount(NULL),
    m_lidar(NULL) {
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
}

std::vector<glm::vec4> OpenLiDAR::scan(float _toDegree, float _atSpeed, bool _verbose) {
    if (_verbose) 
        std::cout << "Start Scanning..." << std::endl;

    double az = 0.0;
    std::vector<glm::vec4> points;
    double start_time = getElapsedSeconds();
    glm::vec3 offset = glm::vec3(0.0,0.0,0.0);
    const std::string deleteLine = "\e[2K\r\e[1A";

    // Start sensor...
    if (m_lidar) {
        m_lidar->start(_verbose);
        offset.x += m_lidar->getHeight();
    }

    // Start motor...
    if (m_mount) {
        offset += m_mount->getOffset();

        // Initialize needed variables to gather data
        size_t count = 0;
        LidarSample samples[RPLIDAR_MAXSAMPLES];
        m_mount->pan(_toDegree, _atSpeed, [&](double _az, double _alt) {
            // calculate time sinze begining of scan
            float delta_time = float(getElapsedSeconds() - start_time);

            // calculate pan quaternion
            glm::quat pan = glm::angleAxis(float(glm::radians(-_az)), glm::vec3(0.0,1.0,0.0));

            if (m_lidar) {
                if (m_lidar->getSamples(samples, count)) {
                    for (size_t i = 0; i < count ; i++) {

                        // calculate tilt quaternion (lidar angle of spinning head)
                        glm::quat tilt = glm::angleAxis(glm::radians(-samples[i].theta), glm::vec3(1.0,0.0,0.0));

                        // calculate 3D position
                        glm::vec3 pos = pan * (tilt * glm::vec3(0.0, 0.0, samples[i].distance) + offset);

                        // Add it to the point buffer
                        points.push_back( glm::vec4(pos, delta_time) );
                    }
                }

            }
            #if defined(DEBUG_USING_SIMULATE_DATA)
            else {
                // SIMULATE DATA
                for (int i = 0; i < 8000 ; i++) {
                    glm::quat tilt = glm::angleAxis(glm::radians(i * 0.045f), glm::vec3(1.0,0.0,0.0));
                    glm::vec3 pos = pan * (tilt * glm::vec3(0.0, 0.0, 1.0) + offset);
                    points.push_back( glm::vec4(pos, delta_time) );
                }
            }
            #endif

            if (_verbose) {
                // Delete previous line
                std::cout << deleteLine;

                // Print progress
                int pct = (_az/_toDegree) * 100;
                std::cout << " [ ";
                for (int i = 0; i < 50; i++) {
                    if (i < pct/2) std::cout << "#";
                    else std::cout << ".";
                }
                std::cout << " ] " << toMMSS(delta_time) << " az: " << toString(_az,1,3,'0') << " pts: " << points.size() << std::endl;
            }
            
            return true;
        });
    }

    // Stop capturing Lidar data
    if (m_lidar)
        m_lidar->stop(_verbose);

    return points;
}

bool OpenLiDAR::reset(bool _verbose) {
    if (_verbose)
        std::cout << "Bringing mount back to 0 azimuth" << std::endl;
        
    if (m_mount)
        return m_mount->reset(_verbose);
    
    #if defined(DEBUG_USING_SIMULATE_DATA)
    return true;
    #endif
    return false;
}
