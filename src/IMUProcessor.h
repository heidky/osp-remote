#pragma once

#ifdef NANO_33_BLE
#include <Arduino_BMI270_BMM150.h>
#endif

#ifdef NANO_33_IOT
#include <Arduino_LSM6DS3.h>
#endif

struct IMUProcessor
{
public:
    float ax, ay, az;
    float gx, gy, gz;

    IMUProcessor() {}
    void begin();
    void update();
};