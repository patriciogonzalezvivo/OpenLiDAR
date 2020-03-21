#include "Scanner.h"

#include "rplidar.h"

#include <fstream>
#include <iostream>
#include <unistd.h>

#include "glm/gtx/transform.hpp"
#include "glm/gtc/quaternion.hpp"
#include "glm/gtx/quaternion.hpp"

#ifndef MOUNT_OFFSET
#define MOUNT_OFFSET_CM 10.0 
#endif

#ifndef _countof
#define _countof(_Array) (int)(sizeof(_Array) / sizeof(_Array[0]))
#endif

// static inline void delay(_word_size_t ms){
//     while (ms>=1000){
//         usleep(1000*1000);
//         ms-=1000;
//     };
//     if (ms!=0)
//         usleep(ms*1000);
// }

using namespace rp::standalone::rplidar;

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
    m_lidar(NULL), 
    m_scanning(false) {

}

Scanner::~Scanner(){

}

bool Scanner::connect(const char* _celestronPort, const char* _rplidarPort) {

    // MOUNT
    // --------------------------------------------------------

    // Connecting to the Celestron Mount
    if (!m_mount.connect(_celestronPort)) {
        std::cerr << "Can't find Celestron Mount in " << _celestronPort << std::endl;
        return false;
    }
    // Get mount information
    FirmwareInfo firmware;
    m_mount.getFirmware(&firmware);

    // Stop any movement on the mount
    m_mount.abort()
    

    //  LIDAR
    // -------------------------------------------------------

    // Connecting to RPLiDAR device
    _u32         baudrateArray[2] = {115200, 256000};
    u_result     op_result;

    bool useArgcBaudrate = false;
    std::cout << "RPLIDAR SDK Version: " << RPLIDAR_SDK_VERSION std::endl;

    // create the driver instance
	m_lidar = RPlidarDriver::CreateDriver(DRIVER_TYPE_SERIALPORT);
    if (!m_lidar) {
        std::cerr << "insufficent memory loading RPLiDAR" << std::endl;
        return false;
    }
    
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
        return false;
    }

    // print out the device serial number, firmware and hardware version number..
    printf("RPLIDAR S/N: ");
    for (int pos = 0; pos < 16 ;++pos) {
        printf("%02X", devinfo.serialnum[pos]);
    }

    printf("\n"
            "Firmware Ver: %d.%02d\n"
            "Hardware Rev: %d\n"
            , devinfo.firmware_version>>8
            , devinfo.firmware_version & 0xFF
            , (int)devinfo.hardware_version);

    // check health...
    if (!checkRPLIDARHealth(m_lidar))
        return false;

    return true;
}

void Scanner::disconnect() {
    m_mount.disconnect();

    if (m_lidar)
        RPlidarDriver::DisposeDriver(m_lidar);
    m_lidar = NULL;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr Scanner::scan(CELESTRON_SLEW_RATE _rate) {
    std::vector<glm::vec3> points;

    // Point the Celestron Mount to 0.0 azimuth 0.0 altitud.
    m_az = 0.0;
    m_alt = 0.0;
    m_mount.slewAzAlt(m_az, m_alt);
    std::cout << "az:  " << az << " alt: " << alt << std::endl;

    // Wait until the mount stop slewing
    while (mount.isSlewing()) {
        usleep(1000);
        m_mount.getAzAlt(&m_az, &m_alt);
        std::cout << "az:  " << m_az << " alt: " << m_alt << std::endl;
    }

    // Move the mount EAST at the speed specify by the user (_rate)
    m_mount.move(CELESTRON_E, _rate);

    // Start motor...
    signal(SIGINT, ctrlc);
    m_lidar->startMotor();

    // start scan...
    m_lidar->startScan(0,1);
    m_scanning = true;

    // fetch result and print it out...
    double az_rad;
    while (m_scanning) {
        // Get mount azimuth angle
        m_mount.getAzAlt(&m_az, &m_alt);
        std::cout << "az:  " << m_az << " alt: " << m_alt << std::endl;
        az_rad = glm::radians(m_az);

        glm::quat lng = glm::angleAxis(az_rad, glm::vec3(0.0,1.0,0.0));
        // glm::quat lat = glm::angleAxis(az_rad, glm::vec3(1.0,1.0,0.0));

        rplidar_response_measurement_node_hq_t nodes[8192];
        size_t   count = _countof(nodes);

        op_result = m_lidar->grabScanDataHq(nodes, count);
        if (IS_OK(op_result)) {
            m_lidar->ascendScanData(nodes, count);
            for (size_t pos = 0; pos < count ; ++pos) {
                double angle = nodes[pos].angle_z_q14 * 90.f / (1 << 14)
                double distance = nodes[pos].dist_mm_q2/4.0f;
                char quality = nodes[pos].quality;

                // printf("%s theta: %03.2f Dist: %08.2f Q: %d \n", 
                //     ( nodes[pos].flag & RPLIDAR_RESP_MEASUREMENT_SYNCBIT ) ?"S ":"  ", 
                //     (   angle ), 
                //         distamce,
                //         quality );
            }
        }

        if (ctrl_c_pressed){
            m_scanning;
        }
    }

    m_lidar->stop();
    m_lidar->stopMotor();


}