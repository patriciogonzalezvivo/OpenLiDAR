#pragma once

#include "../Driver.h"

struct LidarSample {
    float theta;
    float distance;
};

class Lidar : public Driver {
public:
    Lidar(): m_height(0.0) {};
    virtual ~Lidar() {};

    virtual bool    start() = 0;
    virtual bool    stop() = 0;

    virtual bool    getSamples(LidarSample* _samples, size_t& _count) = 0;
    virtual double  getHeight() const { return m_height; }  

protected:
    double  m_height;
};