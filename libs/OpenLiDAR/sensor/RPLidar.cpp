#include "RPLidar.h"

#include <fstream>
#include <iostream>
#include <unistd.h>

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

RPLidar::RPLidar():
m_driver(NULL)
{

}

RPLidar::~RPLidar() {

}

bool RPLidar::connect(const char* _portName) {
    std::cout << "RPLIDAR SDK Version: " << RPLIDAR_SDK_VERSION << std::endl;

    // Attempt to connect at different rates
    _u32   baudrateArray[2] = {115200, 256000};
    size_t baudRateArraySize = (sizeof(baudrateArray))/ (sizeof(baudrateArray[0]));
    for (size_t i = 0; i < baudRateArraySize; ++i) {
        if (!m_driver)
            m_driver = RPlidarDriver::CreateDriver(DRIVER_TYPE_SERIALPORT);

        if (IS_OK(m_driver->connect(_portName, baudrateArray[i]))) {
            rplidar_response_device_info_t devinfo;
            u_result op_result = m_driver->getDeviceInfo(devinfo);

            if (IS_OK(op_result)) {
                m_connected = true;
                break;
            }
            else {
                delete m_driver;
                m_driver = NULL;
            }
        }
    }
    
    if (!m_connected) {
        delete m_driver;
        m_driver = NULL;
    }
    else {
        // check health...
        if (!checkRPLIDARHealth(m_driver)) {
            delete m_driver;
            m_driver = NULL;
        }
    }

    return m_connected;
}

void RPLidar::disconnect() {
    RPlidarDriver::DisposeDriver(m_driver);
    m_driver = NULL;
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


bool RPLidar::start() {
    if (m_driver) {
        // Start motor...
        m_driver->startMotor();

        // start scan...
        m_driver->startScan(0,1);

        return true;
    }

    return false;
}

bool RPLidar::stop() {
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