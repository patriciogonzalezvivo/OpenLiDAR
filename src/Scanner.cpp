#include "Scanner.h"

#include "rplidar.h"

#include <fstream>
#include <iostream>
#include <unistd.h>

#define GLM_ENABLE_EXPERIMENTAL
#include "glm/gtx/transform.hpp"
#include "glm/gtc/quaternion.hpp"
#include "glm/gtx/quaternion.hpp"

#ifndef MOUNT_OFFSET
#define MOUNT_OFFSET_MM 100.0 
#endif

#ifndef _countof
#define _countof(_Array) (int)(sizeof(_Array) / sizeof(_Array[0]))
#endif

bool checkRPLIDARHealth(RPlidarDriver * _drv) {
    u_result     op_result;
    rplidar_response_device_health_t healthinfo;

    op_result = _drv->getHealth(healthinfo);
    if (IS_OK(op_result)) { // the macro IS_OK is the preperred way to judge whether the operation is succeed.
        printf("RPLidar health status : %d\n", healthinfo.status);
        if (healthinfo.status == RPLIDAR_STATUS_ERROR) {
            fprintf(stderr, "Error, rplidar internal error detected. Please reboot the device to retry.\n");
            // enable the following code if you want rplidar to be reboot by software
            // _drv->reset();
            return false;
        } else {
            return true;
        }

    } else {
        fprintf(stderr, "Error, cannot retrieve the lidar health code: %x\n", op_result);
        return false;
    }
}

#include <signal.h>
bool ctrl_c_pressed;
void ctrlc(int)
{
    ctrl_c_pressed = true;
}

Scanner::Scanner() : 
    m_az(0.0),
    m_alt(0.0),
    m_mount(NULL),
    m_lidar(NULL), 
    m_scanning(false) {
}

Scanner::~Scanner(){
    disconnect();
}

bool Scanner::connect(const char* _celestronPort, const char* _rplidarPort) {

    // MOUNT
    // --------------------------------------------------------

    // Connecting to the Celestron Mount
    if (!m_mount) {
        m_mount = new Celestron();

        if (!m_mount->connect(_celestronPort)) {
            std::cerr << "Can't find Celestron Mount in " << _celestronPort << std::endl;
            // return false;
            delete m_mount;
            m_mount = NULL;
        }
        else {
            // Stop any movement on the mount
            m_mount->abort();

            // Get mount information
            FirmwareInfo firmware;
            m_mount->getFirmware(&firmware);
        }
    }
    

    //  LIDAR
    // -------------------------------------------------------

    if (!m_lidar) {
        // Connecting to RPLiDAR device
        _u32         baudrateArray[2] = {115200, 256000};
        u_result     op_result;

        std::cout << "RPLIDAR SDK Version: " << RPLIDAR_SDK_VERSION << std::endl;

        // make connection...
        bool connectSuccess = false;
        rplidar_response_device_info_t devinfo;
        size_t baudRateArraySize = (sizeof(baudrateArray))/ (sizeof(baudrateArray[0]));

        for(size_t i = 0; i < baudRateArraySize; ++i) {

            if (!m_lidar)
                m_lidar = RPlidarDriver::CreateDriver(DRIVER_TYPE_SERIALPORT);

            if (IS_OK(m_lidar->connect(_rplidarPort, baudrateArray[i]))) {
                op_result = m_lidar->getDeviceInfo(devinfo);

                if (IS_OK(op_result)) {
                    connectSuccess = true;
                    break;
                }
                else {
                    delete m_lidar;
                    m_lidar = NULL;
                }
            }
        }
        
        if (!connectSuccess) {
            std::cerr << "Can't find RPLiDAR in " << _rplidarPort << std::endl;
            delete m_lidar;
            m_lidar = NULL;
        }
        else {
            // print out the device serial number, firmware and hardware version number..
            printf("RPLIDAR S/N: ");
            for (int i = 0; i < 16 ;++i)
                printf("%02X", devinfo.serialnum[i]);

            printf("\n"
                    "Firmware Ver: %d.%02d\n"
                    "Hardware Rev: %d\n"
                    , devinfo.firmware_version>>8
                    , devinfo.firmware_version & 0xFF
                    , (int)devinfo.hardware_version);

            // check health...
            if (!checkRPLIDARHealth(m_lidar)) {
                delete m_lidar;
                m_lidar = NULL;
            }
        }
    }

    return (m_lidar != NULL);// && (m_mount != NULL);
}

void Scanner::disconnect() {
    if (m_mount) {
        m_mount->disconnect();
        delete m_mount;
        m_mount = NULL;
    }

    if (m_lidar) {
        RPlidarDriver::DisposeDriver(m_lidar);
        m_lidar = NULL;
    }
}

std::vector<glm::vec3> Scanner::scan(CELESTRON_SLEW_RATE _rate) {
    std::vector<glm::vec3> points;

    // Point the Celestron Mount to 0.0 azimuth 0.0 altitud.
    m_az = 0.0;
    m_alt = 0.0;
    if (m_mount) {
        m_mount->slewAzAlt(m_az, m_alt);
        std::cout << "az:  " << m_az << " alt: " << m_alt << std::endl;

        // Wait until the mount stop slewing
        while (m_mount->isSlewing()) {
            usleep(1000);
            m_mount->getAzAlt(&m_az, &m_alt);
            std::cout << "az:  " << m_az << " alt: " << m_alt << std::endl;
        }

        // Move the mount EAST at the speed specify by the user (_rate)
        m_mount->move(CELESTRON_E, _rate);
    }

    if (m_lidar) {
        // Start motor...
        u_result op_result;
        signal(SIGINT, ctrlc);
        m_lidar->startMotor();

        // start scan...
        m_lidar->startScan(0,1);
        m_scanning = true;

        // fetch result and print it out...
        while (m_scanning && m_az < 180) {

            // Get mount azimuth angle
            if (m_mount) m_mount->getAzAlt(&m_az, &m_alt);
            else m_az += 1.;

            std::cout << "az:  " << m_az << " alt: " << m_alt << std::endl;
            glm::quat lng = glm::angleAxis(float(glm::radians(m_az)), glm::vec3(0.0,1.0,0.0));

            rplidar_response_measurement_node_hq_t nodes[8192];
            size_t   count = _countof(nodes);

            op_result = m_lidar->grabScanDataHq(nodes, count);
            if (IS_OK(op_result)) {
                m_lidar->ascendScanData(nodes, count);
                for (size_t i = 0; i < count ; ++i) {
                    float angle = nodes[i].angle_z_q14 * 90.f / (1 << 14);
                    float distance = nodes[i].dist_mm_q2 / 1000.f / (1 << 2);
                    char quality = nodes[i].quality;

                    // if (quality >= 127) 
                    {
                        glm::quat lat = glm::angleAxis(glm::radians(angle), glm::vec3(1.0,0.0,0.0));
                        glm::vec3 pos = lng * (lat * glm::vec3(0.0, 0.0, distance));

                        points.push_back(pos);
                    }

                    // printf("%s theta: %03.2f Dist: %08.2f Q: %d \n", 
                    //     ( nodes[i].flag & RPLIDAR_RESP_MEASUREMENT_SYNCBIT ) ?"S ":"  ", 
                    //     (   angle ), 
                    //         distance,
                    //         quality );

                }
            }

            if (ctrl_c_pressed)
                m_scanning = false;
        }

        m_lidar->stop();
        m_lidar->stopMotor();
    }

    return points;
}