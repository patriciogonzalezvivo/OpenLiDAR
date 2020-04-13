#pragma once

#include "../Driver.h"

#include <glm/glm.hpp>

enum ImuType {
    BERRY = 0,
};

class ImuDriver : public Driver {
public:
    ImuDriver(): m_acc(0.0), m_gyr(0.0), m_heading(0.0) {};
    virtual ~ImuDriver() {};

    virtual bool    start(bool _verbose) = 0;
    virtual void    update() = 0;
    virtual bool    stop(bool _verbose) = 0;

    virtual bool    calibrate(bool _verbose) = 0;

    virtual glm::vec3   getAcc() { return m_acc; }
    virtual glm::vec3   getGyr() { return m_gyr; }
    virtual float       getHeading() { return m_heading; }

protected:
    glm::vec3   m_acc;
    glm::vec3   m_gyr;
    float       m_heading;
};