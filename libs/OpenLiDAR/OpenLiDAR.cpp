#include "OpenLiDAR.h"

#include <unistd.h>
#include <iostream>

#include "tools.h"

#define GLM_ENABLE_EXPERIMENTAL
#include "glm/gtx/transform.hpp"
#include "glm/gtc/quaternion.hpp"
#include "glm/gtx/quaternion.hpp"

#include "mount/Celestron.h"
#include "lidar/RPLidar.h"

OpenLiDAR::OpenLiDAR() :
    m_mount(NULL),
    m_lidar(NULL), 
    m_scanning(false) {
}

OpenLiDAR::~OpenLiDAR(){
    disconnect();
}

bool OpenLiDAR::connect(LidarType _lidarType, MountType _mountType, bool _verbose) {

    if (!m_lidar) {
        switch (_lidarType)
        {
        case RPLIDAR:
            m_lidar = new RPLidar();
            break;
        
        default:
            break;
        }
        m_lidar = new RPLidar();
    }

    if (!m_mount) {
        switch (_mountType) {
        case CELESTRON:
            m_mount = new Celestron();
            break;
        
        default:
            break;
        }
    }

    char* _mountPort = m_mount->getPort();
    char* _lidarPort = m_lidar->getPort();
    
    if (_verbose) {
        std::cout << "Mount found at " << _mountPort << std::endl;
        std::cout << "Lidar found at " << _lidarPort << std::endl;
    }
    
    if (!m_mount->connect(_mountPort, _verbose)) {
        std::cerr << "Can't load Mount from " << _mountPort << std::endl;
        delete m_mount;
        m_mount = NULL;
    }

    if (!m_lidar->connect(_lidarPort, _verbose)) {
        std::cerr << "Can't load Sensor from " << _lidarPort << std::endl;
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

bool OpenLiDAR::connect(const char* _lidarPort, const char* _mountPort, bool _verbose) {

    // MOUNT
    // --------------------------------------------------------

    // Connecting to the Celestron Mount
    if (!m_mount) {
        m_mount = new Celestron();

        if (!m_mount->connect(_mountPort, _verbose)) {
            std::cerr << "Can't find Celestron Mount in " << _mountPort << std::endl;
            delete m_mount;
            m_mount = NULL;
        }
    }
    
    //  LIDAR
    // -------------------------------------------------------
    if (!m_lidar) {
        m_lidar = new RPLidar();

        if (!m_lidar->connect(_lidarPort, _verbose)) {
            std::cerr << "Can't find RPLidar Sensor in " << _lidarPort << std::endl;
            delete m_lidar;
            m_lidar = NULL;
        }
    }

#if defined(DEBUG_USING_SIMULATE_DATA)
    if (m_lidar == NULL || m_mount == NULL)
        std::cout << "WARNING!!! Basic devices are not connected, data will be simulated." << std::endl;

    return true;
#endif

    return (m_lidar != NULL) && (m_mount != NULL);
}

void OpenLiDAR::disconnect() {
    if (m_mount) {
        m_mount->disconnect();
        delete m_mount;
        m_mount = NULL;
    }

    if (m_lidar) {
        m_lidar->disconnect();
        delete m_lidar;
        m_lidar = NULL;
    }
}

std::vector<glm::vec4> OpenLiDAR::scan(float _toDegree, float _atSpeed, bool _verbose) {
    if (_verbose) 
        std::cout << "Start Scanning" << std::endl;

    m_scanning = true;
    glm::vec3 offset = glm::vec3(0.0,0.0,0.0);
    double az = 0.0;
    double start_time = getElapsedSeconds();
    std::vector<glm::vec4> points;

    // Start motor...
    if (m_mount) {
        m_mount->start(_atSpeed, _verbose);
        offset += m_mount->getOffset();
    }

    // Start sensor...
    if (m_lidar) {
        m_lidar->start();
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
        // SIMULATE MOUNT DATA
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
            // SIMULATE LIDAR DATA
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
        m_lidar->stop();

    // Stop panning 
    if (m_mount)
        m_mount->stop();

    return points;
}

bool OpenLiDAR::reset(bool _verbose) {
    if (_verbose)
        std::cout << "Reset scanner" << std::endl;

    if (m_mount)
        return m_mount->reset(_verbose);
    
    #if defined(DEBUG_USING_SIMULATE_DATA)
    return true;
    #endif
    return false;
}
