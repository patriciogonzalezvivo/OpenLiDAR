#pragma once

#include <iostream>

#ifndef TOTAL_PORTS 
#define TOTAL_PORTS 2
#endif

static char* ports[TOTAL_PORTS] = {
    (char*)"/dev/ttyUSB0", 
    (char*)"/dev/ttyUSB1"
};

class Driver {
public:
    Driver(): m_connected(false) {}
    virtual ~Driver() {};

    virtual bool    autoconnect() {
        for (int i = 0; i < TOTAL_PORTS; i++) {
            if (connect(ports[i])) {
                return true;
                break;
            }
        }
        return false;
    };

    virtual bool    connect(const char* _portName) = 0;
    virtual void    disconnect() = 0;
    virtual bool    printFirmware() = 0;

protected:
    bool m_connected;

};
