#pragma once

#include <functional>
#include <glm/glm.hpp>

#include "../Driver.h"

enum MountType {
    CELESTRON = 0
};

class MountDriver : public Driver {
public:
    MountDriver() : m_offset(0.0,0.0,0.0), m_az(0.f), m_alt(0.f) {};
    virtual ~MountDriver() {};

    virtual bool    pan(double _targetAngle, float _speed, std::function<bool(double&, double&)> _callback) = 0;
    virtual bool    reset(bool _verbose) = 0;

    virtual double      getAz() { return m_az; }
    virtual double      getAlt() { return m_alt; };
    virtual glm::vec3   getOffset() { return m_offset; }

protected:
    glm::vec3   m_offset;
    double      m_az;
    double      m_alt;
};