#pragma once

#include "../Driver.h"

enum LidarType {
    RPLIDAR = 0,
};

struct LidarSample {
    float theta;
    float distance;
};

class LidarDriver : public Driver {
public:
    LidarDriver(): m_height(0.0) {};
    virtual ~LidarDriver() {};

    virtual bool    start(bool _verbose) = 0;
    virtual bool    stop(bool _verbose) = 0;

    virtual bool    getSamples(LidarSample* _samples, size_t& _count) = 0;
    virtual double  getHeight() const { return m_height; }  

protected:
    double  m_height;
};