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
        if (_verbose) {
            std::cout << " Health status: " ;

            switch (healthinfo.status) {
                case RPLIDAR_STATUS_OK:
                    std::cout << "OK." << std::endl;
                    break;
                case RPLIDAR_STATUS_WARNING:
                    std::cout << "Warning." << std::endl;
                    break;
                case RPLIDAR_STATUS_ERROR:
                    std::cout << "Error." << std::endl;
                    break;
            }
        }
            
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
    bool connectSuccess = false;
    rplidar_response_device_info_t devinfo;
    size_t baudRateArraySize = (sizeof(baudrateArray))/ (sizeof(baudrateArray[0]));
    for (size_t i = 0; i < baudRateArraySize; ++i) {
        if (!m_driver)
            m_driver = RPlidarDriver::CreateDriver(DRIVER_TYPE_SERIALPORT);

        if (IS_OK(m_driver->connect(_portName, baudrateArray[i]))) {
            op_result = m_driver->getDeviceInfo(devinfo);

            if (IS_OK(op_result)) {
                connectSuccess = true;
                break;
            }
            else {
                delete m_driver;
                m_driver = NULL;
            }
        }

        if (m_connected) {
            if (_verbose)
                std::cout << "Successfully connected to " << _portName << " at " << baudrateArray[i] << std::endl;
            break;
        }
        else
            disconnect();
    }

    if (!connectSuccess) {
        if (_verbose)
            std::cerr << "Error, cannot bind LiDAR driver to the specified serial port " << _portName << std::endl;
        disconnect();
    }
    else {
        m_connected = true;

        if (_verbose)
            printFirmware();

        // check health...
        if (!checkRPLIDARHealth(m_driver, _verbose)) {
            std::cerr << "Something went wrong with RPLiDAR health. Disconnecting" << std::endl;
            disconnect();
        }

        if (m_driver)
            m_driver->startMotor();
    }

    return m_connected;
}

void RPLidar::disconnect() {
    m_connected = false;

    if (m_driver == NULL)
        return;

    m_driver->stopMotor();
    RPlidarDriver::DisposeDriver(m_driver);
    m_driver = NULL;
}

bool RPLidar::printFirmware() {
    if (m_driver == NULL) 
        return false;

    rplidar_response_device_info_t devinfo;
    u_result op_result = m_driver->getDeviceInfo(devinfo);

    if (IS_OK(op_result)) {
        std::cout << "RPLIDAR S/N: ";
        for (int i = 0; i < 16 ;++i)
            printf("%02X", devinfo.serialnum[i]);

        printf( "\n"
                " Firmware Ver: %d.%02d\n"
                " Hardware Rev: %d\n"
                , devinfo.firmware_version>>8
                , devinfo.firmware_version & 0xFF
                , (int)devinfo.hardware_version);
    }

    return true;
}


bool RPLidar::start(bool _verbose) {
    if (m_driver == NULL) 
        return false;
        
    if (_verbose)
        std::cout << "Start collecting LiDAR data" << std::endl;

    m_driver->startScan(0,1);
    return true;
}

bool RPLidar::stop(bool _verbose) {
    if (m_driver == NULL)
        return false;

    m_driver->stop();
    return true;
}


bool RPLidar::getSamples(LidarSample* _samples, size_t& _count) {
    if (m_driver == NULL)
        return false;
        
    _count = _countof(m_nodes);
    u_result op_result = m_driver->grabScanDataHq(m_nodes, _count);
    if (IS_OK(op_result)) {
        // std::cout << "got Hq data" << std::endl;
        m_driver->ascendScanData(m_nodes, _count);
        for (size_t i = 0; i < _count ; ++i) {
            _samples[i].theta = m_nodes[i].angle_z_q14 * 90.f / (1 << 14); 
            _samples[i].distance = m_nodes[i].dist_mm_q2 / 1000.f / (1 << 2); // Meters
            // char quality = m_nodes[i].quality;
        }
        return true;
    }

    return false;
}