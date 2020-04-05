#include "RPLidar.h"

#include <fstream>
#include <iostream>
#include <unistd.h>

#ifndef _countof
#define _countof(_Array) (int)(sizeof(_Array) / sizeof(_Array[0]))
#endif

bool checkRPLIDARHealth(RPlidarDriver * _drv, bool _verbose) {
    u_result     op_result;
    rplidar_response_device_health_t healthinfo;

    op_result = _drv->getHealth(healthinfo);
    if (IS_OK(op_result)) { // the macro IS_OK is the preperred way to judge whether the operation is succeed.
        if (_verbose)
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

RPLidar::RPLidar():
m_driver(NULL)
{
}

RPLidar::~RPLidar() {
    disconnect();
}

bool RPLidar::connect(const char* _portName, bool _verbose) {

    if (m_connected)  {
        std::cout << "RPLidar is already connected." << std::endl;
        return false;
    }

    // Connecting to RPLiDAR device
    _u32         baudrateArray[2] = {115200, 256000};
    u_result     op_result;

    // std::cout << "RPLIDAR SDK Version: " << RPLIDAR_SDK_VERSION << std::endl;

    // make connection...
    rplidar_response_device_info_t devinfo;
    size_t baudRateArraySize = (sizeof(baudrateArray))/ (sizeof(baudrateArray[0]));
    for (size_t i = 0; i < baudRateArraySize; ++i) {
        if (!m_driver)
            m_driver = RPlidarDriver::CreateDriver(DRIVER_TYPE_SERIALPORT);

        if (IS_OK(m_driver->connect(_portName, baudrateArray[i]))) {
            op_result = m_driver->getDeviceInfo(devinfo);
            m_connected = IS_OK(op_result);
        }

        if (m_connected)
            break;
        else
            disconnect();
    }
    
    if (m_connected) {
        if (_verbose)
            printFirmware();

        // check health...
        if (!checkRPLIDARHealth(m_driver, _verbose))
            disconnect();
    }

    return m_connected;
}

void RPLidar::disconnect() {
    if (m_driver) {
        delete m_driver;
        m_driver = NULL;
    }

    m_connected = false;
}

bool RPLidar::printFirmware() {
    if (m_driver) {
        rplidar_response_device_info_t devinfo;
        u_result op_result = m_driver->getDeviceInfo(devinfo);

        if (IS_OK(op_result)) {
            printf("RPLIDAR S/N: ");
            for (int i = 0; i < 16 ;++i)
                printf("%02X", devinfo.serialnum[i]);

            printf("\n"
                    "Firmware Ver: %d.%02d\n"
                    "Hardware Rev: %d\n"
                    , devinfo.firmware_version>>8
                    , devinfo.firmware_version & 0xFF
                    , (int)devinfo.hardware_version);
        }
    }

    return false;
}


bool RPLidar::start(bool _verbose) {
    if (m_driver) {
        if (_verbose)
            std::cout << "Start LiDAR motor" << std::endl;
        m_driver->startMotor();

        if (_verbose)
            std::cout << "Start collecting LiDAR data" << std::endl;
        m_driver->startScan(0,1);

        return true;
    }

    return false;
}

bool RPLidar::stop(bool _verbose) {
    if (m_driver) {
        m_driver->stop();
        m_driver->stopMotor();

        return true;
    }
    return false;
}


bool RPLidar::getSamples(LidarSample* _samples, size_t& _count) {
    if (m_driver) {
        _count = _countof(m_nodes);

        u_result op_result = m_driver->grabScanDataHq(m_nodes, _count);
        if (IS_OK(op_result)) {
            m_driver->ascendScanData(m_nodes, _count);
            for (size_t i = 0; i < _count ; ++i) {
                _samples[i].theta = m_nodes[i].angle_z_q14 * 90.f / (1 << 14); 
                _samples[i].distance = m_nodes[i].dist_mm_q2 / 1000.f / (1 << 2); // Meters
                // char quality = m_nodes[i].quality;
            }

            return true;
        }
    }
    return false;
}