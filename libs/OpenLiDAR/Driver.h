#pragma once

#include <iostream>

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
