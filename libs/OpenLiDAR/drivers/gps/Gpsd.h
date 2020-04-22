#pragma once

#include <vector>

#include "GpsDriver.h"

#include <libgpsmm.h>

class Gpsd : public GpsDriver {
public:

    Gpsd();
    virtual ~Gpsd();

    bool        connect(const char* _portName = "localhost", bool _verbose = true); // Default: 
    void        disconnect();

    bool        printFirmware();

    void        update();

    double      getLat();
    double      getLng();
    double      getAlt();

protected:
    std::vector<double> m_lats;
    std::vector<double> m_lngs;
    std::vector<double> m_alts;

    gpsmm*      m_gps;
};