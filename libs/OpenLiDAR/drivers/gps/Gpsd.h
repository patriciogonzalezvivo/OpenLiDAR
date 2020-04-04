#pragma once

#include <mutex>
#include <thread>
#include <vector>

#include "GpsDriver.h"

#include <libgpsmm.h>

class Gpsd : public GpsDriver {
public:

    Gpsd();
    virtual ~Gpsd();

    bool        connect(const char* _portName, bool _verbose); // Default: "localhost"
    void        disconnect();

    bool        printFirmware();

    bool        start(bool _verbose);
    bool        stop(bool _verbose);

    double      getLat();
    double      getLng();
    double      getAlt();

protected:
    std::vector<double> m_lats;
    std::vector<double> m_lngs;
    std::vector<double> m_alts;

    std::thread m_thread;
    std::mutex  m_mutex;
    gpsmm*      m_gps;
};