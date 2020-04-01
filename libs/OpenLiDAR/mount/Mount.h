#pragma once

#include <glm/glm.hpp>

#include "../Driver.h"

class Mount : public Driver {
public:
    Mount() : m_offset(0.08, 0.0, -0.12), m_az(0.f), m_alt(0.f) {};
    virtual ~Mount() {};

    virtual bool    start(float _speed, bool _verbose) = 0;
    virtual bool    stop() = 0;
    virtual bool    update() = 0;
    virtual bool    reset(bool _verbose) = 0;

    virtual double  getAz() { 
        update();
        return m_az; 
    }

    virtual double  getAlt() { 
        update();
        return m_alt; 
    };

    virtual glm::vec3 getOffset() const { return m_offset; }

protected:
    glm::vec3   m_offset;
    double      m_az;
    double      m_alt;
};