#pragma once

#include "../Driver.h"

enum GpsType {
    GPSD = 0,
};

class GpsDriver : public Driver {
public:
    GpsDriver(): m_lat(0.0), m_lng(0.0), m_alt(0.0) {};
    virtual ~GpsDriver() {};

    virtual void    update() = 0;

    virtual double  getLat() { return m_lat; }
    virtual double  getLng() { return m_lng; }
    virtual double  getAlt() { return m_alt; }

protected:
    double  m_lat;
    double  m_lng;
    double  m_alt;
};