#pragma once

#include "ImuDriver.h"

class BerryIMU : public ImuDriver {
public:

    BerryIMU();
    virtual ~BerryIMU();

    bool    connect(const char* _portName = "/dev/i2c-%d", bool _verbose = true);
    void    disconnect();

    bool    printFirmware();

    void    update();

    bool    calibrate(bool _start);

protected:
    void        readBlock(uint8_t _command, uint8_t _size, uint8_t *_data);
    void        selectDevice(int _file, int _addr);

    void        writeAccReg(uint8_t _reg, uint8_t _value);
    void        writeMagReg(uint8_t _reg, uint8_t _value);
    void        writeGyrReg(uint8_t _reg, uint8_t _value);

    void        readACC(int _a[]);
    void        readMAG(int _m[]);
    void        readGYR(int _g[]);

    void        updateAccGyr();
    void        updateMag();

    glm::ivec3  m_magMax;
    glm::ivec3  m_magMin;

    double      m_prevTime;

    int         m_file;
    bool        m_LSM9DS0;
    bool        m_LSM9DS1;
    bool        m_calibrating;
};