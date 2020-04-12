#pragma once

#include "ImuDriver.h"

class BerryIMU : public ImuDriver {
public:

    BerryIMU();
    virtual ~BerryIMU();

    bool    connect(const char* _portName, bool _verbose); // Default: "localhost"
    void    disconnect();

    bool    calibrate();
    bool    printFirmware();

    bool    start(bool _verbose);
    bool    stop(bool _verbose);

protected:

};