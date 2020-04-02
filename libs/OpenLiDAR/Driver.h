#pragma once

#include <iostream>

#ifndef PORTS_TOTAL 
#define PORTS_TOTAL 4
#endif

static char* PORTS[PORTS_TOTAL] = {
    (char*)"/dev/ttyUSB0", 
    (char*)"/dev/ttyUSB1",
    (char*)"/dev/ttyUSB2", 
    (char*)"/dev/ttyUSB3"
};

class Driver {
public:
    Driver(): m_connected(false) {}
    virtual ~Driver() {};

    virtual bool    connect(const char* _portName, bool _verbose) = 0;
    virtual void    disconnect() = 0;
    virtual bool    printFirmware() = 0;

    virtual char*   getPort() {
        for (int i = 0; i < PORTS_TOTAL; i++) {
            bool success = connect(PORTS[i], false);
            disconnect();

            if (success)
                return PORTS[i];
        }

        return (char*)"UNKNOWN";
    }

protected:
    bool m_connected;

};
