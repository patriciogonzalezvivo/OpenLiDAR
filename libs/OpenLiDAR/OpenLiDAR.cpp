#include "OpenLiDAR.h"

#include <unistd.h>
#include <iostream>

#include "tools.h"

#define GLM_ENABLE_EXPERIMENTAL
#include "glm/gtx/transform.hpp"
#include "glm/gtc/quaternion.hpp"
#include "glm/gtx/quaternion.hpp"

// #ifndef TOTAL_PORTS 
// #define TOTAL_PORTS 2
// #endif

// static char* ports[TOTAL_PORTS] = {
//     (char*)"/dev/ttyUSB0", 
//     (char*)"/dev/ttyUSB1"
// };

OpenLiDAR::OpenLiDAR() : 
    m_offset(0.08, 0.0, -0.12),
    m_az(0.0),
    m_alt(0.0),
    m_mount(NULL),
    m_lidar(NULL), 
    m_scanning(false) {
}

OpenLiDAR::~OpenLiDAR(){
    disconnect();
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

std::vector<glm::vec4> OpenLiDAR::scan(float _degree, float _speed, bool _verbose) {
    CELESTRON_SLEW_RATE rate = CELESTRON_SLEW_RATE(ceil(_speed * SR_9));
    double start_time = getElapsedSeconds();

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
        if (_verbose)
            std::cout << "Start Scanning" << std::endl;

        // Start motor...
        m_lidar->start();
        m_scanning = true;

        LidarSample samples[RPLIDAR_MAXSAMPLES];
        size_t   count;

        // fetch result and print it out...
        while (m_scanning && m_az < _degree) {
            float time = float(getElapsedSeconds() - start_time);

            // Get mount azimuth angle
            m_mount->getAzAlt(&m_az, &m_alt);
            glm::quat lng = glm::angleAxis(float(glm::radians(-m_az)), glm::vec3(0.0,1.0,0.0));

            if (m_lidar->getSamples(samples, count)) {
                for (size_t i = 0; i < count ; ++i) {
                    glm::quat lat = glm::angleAxis(glm::radians(-samples[i].theta), glm::vec3(1.0,0.0,0.0));
                    glm::vec3 pos = lng * (lat * glm::vec3(0.0, 0.0, samples[i].distance) + m_offset);
                    points.push_back( glm::vec4(pos, time) );
                }
            }

            if (_verbose) {
                // Delete previous line
                const std::string deleteLine = "\e[2K\r\e[1A";
                std::cout << deleteLine;

                int pct = (m_az/_degree) * 100;
                
                std::cout << " [ ";
                for (int i = 0; i < 50; i++) {
                    if (i < pct/2) {
                        std::cout << "#";
                    }
                    else {
                        std::cout << ".";
                    }
                }
                std::cout << " ] " << toMMSS(time) << " az: " << toString(m_az,1,3,'0') << " alt: " << toString(m_alt,1,3,'0') << " pts: " << points.size() << std::endl;
            }
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

bool OpenLiDAR::reset(bool _verbose) {
    if (m_mount) {
        double start_time = getElapsedSeconds();
        double start_az = m_az;

        if (_verbose)
            std::cout << "Moving mount to original Azimuthal angle" << std::endl;

        m_mount->move(CELESTRON_E, SR_9);
        while (m_az > 5.) {

            usleep(1000);
            m_mount->getAzAlt(&m_az, &m_alt);

            if (_verbose) {
                // Delete previous line
                const std::string deleteLine = "\e[2K\r\e[1A";
                std::cout << deleteLine;

                int pct = (1.0 - m_az/start_az) * 100;
                float time = float(getElapsedSeconds() - start_time);
                
                std::cout << " [ ";
                for (int i = 0; i < 50; i++) {
                    if (i < pct/2) {
                        std::cout << "#";
                    }
                    else {
                        std::cout << ".";
                    }
                }
                std::cout << " ] " << toMMSS(time) << " az: " << toString(m_az,1,3,'0') << " alt: " << toString(m_alt,1,3,'0') << std::endl;
            }
        }
        m_mount->stop(CELESTRON_E);
    }
    return true;
}
