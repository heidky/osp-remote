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

protected:
    float SR;

    float strength = 0;

    Adafruit_Madgwick filter;
};