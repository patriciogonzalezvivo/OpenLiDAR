#include "OpenLiDAR.h"

#include <unistd.h>
#include <iostream>

#include "tools.h"

#define GLM_ENABLE_EXPERIMENTAL
#include "glm/gtx/transform.hpp"
#include "glm/gtc/quaternion.hpp"
#include "glm/gtx/quaternion.hpp"

#ifndef MOUNT_OFFSET_X
#define MOUNT_OFFSET_X 0.06
#endif

#ifndef MOUNT_OFFSET_Y
#define MOUNT_OFFSET_Y 0.0
#endif

#ifndef MOUNT_OFFSET_Z
#define MOUNT_OFFSET_Z -0.12
#endif

#ifndef MOUNT_TURN
#define MOUNT_TURN 200.0
#endif

#ifndef TOTAL_PORTS 
#define TOTAL_PORTS 2
#endif

static char* ports[TOTAL_PORTS] = {
    (char*)"/dev/ttyUSB0", 
    (char*)"/dev/ttyUSB1"
};

OpenLiDAR::OpenLiDAR() : 
    m_az(0.0),
    m_alt(0.0),
    m_mount(NULL),
    m_lidar(NULL), 
    m_scanning(false) {
}

OpenLiDAR::~OpenLiDAR(){
    disconnect();
}

bool OpenLiDAR::connect() {

    int mount_port = -1;
    int lidar_port = -1;

    // MOUNT
    // --------------------------------------------------------

    // Connecting to the Celestron Mount
    if (!m_mount) {
        m_mount = new Celestron();

        for (int i = 0; i < TOTAL_PORTS; i++) {
            if (m_mount->connect(ports[i])) {
                mount_port = i;
                // m_mount->printFirmware();
                break;
            }
        }

        if (mount_port == -1) {
            std::cerr << "Can't find Celestron Mount port" << std::endl;
            delete m_mount;
            m_mount = NULL;
        }
    }
    
    //  LIDAR
    // -------------------------------------------------------
    if (!m_lidar) {
        m_lidar = new RPLidar();
        for (int i = 0; i < TOTAL_PORTS; i++) {
            if (mount_port != i) {
                if (m_lidar->connect(ports[i])) {
                    lidar_port = i;
                    // m_lidar->printFirmware();
                    break;
                }
            }
        }

        if (lidar_port == -1) {
            std::cerr << "Can't find Sensor port" << std::endl;
            delete m_lidar;
            m_lidar = NULL;
        }        
    }

    return (m_lidar != NULL) && (m_mount != NULL);
}

bool OpenLiDAR::connect(const char* _celestronPort, const char* _rplidarPort) {

    // MOUNT
    // --------------------------------------------------------

    // Connecting to the Celestron Mount
    if (!m_mount) {
        m_mount = new Celestron();

        if (!m_mount->connect(_celestronPort)) {
            std::cerr << "Can't find Celestron Mount in " << _celestronPort << std::endl;
            delete m_mount;
            m_mount = NULL;
        }
        // else
        //     m_mount->printFirmware();
    }
    
    //  LIDAR
    // -------------------------------------------------------
    if (!m_lidar) {
        m_lidar = new RPLidar();

        if (!m_lidar->connect(_rplidarPort)) {
            std::cerr << "Can't find RPLidar Sensor in " << _rplidarPort << std::endl;
            delete m_lidar;
            m_lidar = NULL;
        }
        // else 
        //     m_lidar->printFirmware();
    }

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

std::vector<glm::vec4> OpenLiDAR::scan(float _loop, float _speed) {
    CELESTRON_SLEW_RATE rate = CELESTRON_SLEW_RATE(ceil(_speed * SR_9));
    int max_angle = ceil(360 * _loop);
    double time_start = getElapsedSeconds();

    std::vector<glm::vec4> points;

    m_az = 0.0;
    m_alt = 0.0;
    if (m_mount) {
        double az, alt;
        m_mount->getAzAlt(&az, &alt);

        // Point the Celestron Mount to 0.0 azimuth 0.0 altitud.
        if ((az != m_az) && (alt != m_alt))
            m_mount->gotoAzAlt(m_az, m_alt);

        // Move the mount WEST at the speed specify by the user (rate)
        m_mount->move(CELESTRON_W, rate);
    }
    else
        std::cout << "Mount connection lost" << std::endl;

    if (m_lidar) {
        // Start motor...
        m_lidar->start();
        m_scanning = true;

        LidarSample samples[RPLIDAR_MAXSAMPLES];
        size_t   count;

        // fetch result and print it out...
        while (m_scanning && m_az < max_angle) {
            // Get mount azimuth angle
            m_mount->getAzAlt(&m_az, &m_alt);
            glm::quat lng = glm::angleAxis(float(glm::radians(-m_az)), glm::vec3(0.0,1.0,0.0));

            if (m_lidar->getSamples(samples, count)) {
                for (size_t i = 0; i < count ; ++i) {
                    glm::quat lat = glm::angleAxis(glm::radians(-samples[i].theta), glm::vec3(1.0,0.0,0.0));
                    glm::vec3 pos = lng * (lat * glm::vec3(0.0, 0.0, samples[i].distance) + glm::vec3(MOUNT_OFFSET_X, MOUNT_OFFSET_Y, MOUNT_OFFSET_Z));
                    float time = getElapsedSeconds() - time_start;
                    points.push_back( glm::vec4(pos, time) );
                }
            }


            // Delete previous line
            const std::string deleteLine = "\e[2K\r\e[1A";
            std::cout << deleteLine;

            int pct = (m_az/max_angle) * 100;
            
            std::cout << " [ ";
            for (int i = 0; i < 50; i++) {
                if (i < pct/2) {
                    std::cout << "#";
                }
                else {
                    std::cout << ".";
                }
            }
            std::cout << " ] az: " << m_az << " alt: " << m_alt << " pts: " << points.size() << std::endl;
        }

        m_lidar->stop();

        // Stop any movement on the mount
        if (m_mount)
            m_mount->stop(CELESTRON_W);
    }
    else
        std::cout << "Sensor connection lost" << std::endl;

    return points;
}

bool OpenLiDAR::reset() {
    if (m_mount) {
        m_mount->move(CELESTRON_E, SR_9);
        while (m_az > 5.) {
            usleep(1000);
            m_mount->getAzAlt(&m_az, &m_alt);
        }
        m_mount->stop(CELESTRON_E);
    }
    return true;
}
