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

    virtual bool    connect() {
        for (int i = 0; i < TOTAL_PORTS; i++) {
            if (connect(ports[i])) {
                return true;
                break;
            }
        }
        return false;
    };

    virtual bool    connect(const char* _portName) { 
        std::cout << "IMPLEMENT CONNECT" << std::endl;
        return false; 
    };
    virtual void    disconnect() {};
    virtual bool    printFirmware() { return false; };

protected:
    bool m_connected;

};
