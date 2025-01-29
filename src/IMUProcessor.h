#pragma once

#include <Adafruit_Sensor.h>
#include <Adafruit_AHRS.h>

struct IMUProcessor
{
public:
    float ax, ay, az;
    float gx, gy, gz;

    IMUProcessor() {}
    void begin(float SR);
    void update();

    inline float get_strength() { return strength; }
    inline void set_strength(float s) { strength = constrain(s, 0, 1); }

protected:
    float SR;

    float strength = 0;

    Adafruit_Madgwick filter;
};