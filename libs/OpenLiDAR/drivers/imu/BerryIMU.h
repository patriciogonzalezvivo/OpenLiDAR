#pragma once

#include "ImuDriver.h"

class BerryIMU : public ImuDriver {
public:

    BerryIMU();
    virtual ~BerryIMU();

    bool    connect(const char* _portName = "/dev/i2c-%d", bool _verbose = true);
    void    disconnect();

    bool    calibrate();
    bool    printFirmware();

    bool    start(bool _verbose);
    void    update();
    bool    stop(bool _verbose);

protected:
    void    readBlock(uint8_t _command, uint8_t _size, uint8_t *_data);
    void    selectDevice(int _file, int _addr);
    void    enableIMU();
    void    readACC(int _a[]);
    void    readMAG(int _m[]);
    void    readGYR(int _g[]);
    void    writeAccReg(uint8_t _reg, uint8_t _value);
    void    writeMagReg(uint8_t _reg, uint8_t _value);
    void    writeGyrReg(uint8_t _reg, uint8_t _value);

    int     m_file;
    bool    m_LSM9DS0;
    bool    m_LSM9DS1;
};