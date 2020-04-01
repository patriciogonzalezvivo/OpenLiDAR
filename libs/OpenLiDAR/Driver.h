#pragma once

#include <iostream>

// #ifndef TOTAL_PORTS 
// #define TOTAL_PORTS 4
// #endif

// static char* ports[TOTAL_PORTS] = {
//     (char*)"/dev/ttyUSB0", 
//     (char*)"/dev/ttyUSB1"
//     (char*)"/dev/ttyUSB2", 
//     (char*)"/dev/ttyUSB3"
// };

class Driver {
public:
    Driver(): m_connected(false) {}
    virtual ~Driver() {};

    virtual bool    connect(const char* _portName) = 0;
    virtual void    disconnect() = 0;
    virtual bool    printFirmware() = 0;

protected:
    bool m_connected;

};
