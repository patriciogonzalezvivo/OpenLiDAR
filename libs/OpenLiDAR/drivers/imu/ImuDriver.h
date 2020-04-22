#pragma once

#include "../Driver.h"

#include <glm/glm.hpp>

enum ImuType {
    BERRY = 0,
};

class ImuDriver : public Driver {
public:
    ImuDriver(): m_acc(0.0), m_gyr(0.0), m_roll(0.0), m_pitch(0.0), m_heading(0.0) {};
    virtual ~ImuDriver() {};

    virtual void    update() = 0;

    virtual bool    calibrate(bool _verbose) = 0;

    virtual glm::vec3   getAcc() { return m_acc; }
    virtual glm::vec3   getGyr() { return m_gyr; }

    virtual float       getRoll() { return m_roll; }
    virtual float       getPitch() { return m_pitch; }
    virtual float       getHeading() { return m_heading; }

protected:
    glm::vec3   m_acc;
    glm::vec3   m_gyr;

    float       m_roll;
    float       m_pitch;
    float       m_heading;
};