#include "IMUProcessor.h"
#include "log.h"

void IMUProcessor::begin(float SR)
{
    this->SR = SR;
    filter.setBeta(0.5);
    filter.begin(SR);
}

void IMUProcessor::update()
{
    // LOG_(">ax:"); LOG(ax);
    // LOG_(">ay:"); LOG(ay);
    // LOG_(">az:"); LOG(az);
    // LOG_(">gx:"); LOG(gx);
    // LOG_(">gy:"); LOG(gy);
    // LOG_(">gz:"); LOG(gz);

    filter.updateIMU(gx, gy, gz, ax, ay, az);

    float qw, qx, qy, qz;
    filter.getQuaternion(&qw, &qx, &qy, &qz);
    float gravity_x = 2.0f * (qx*qz - qw*qy);
    float gravity_y = 2.0f * (qw*qx + qy*qz);
    float gravity_z = qw*qw - qx*qx - qy*qy + qz*qz;

    LOG_(">grav_x:"); LOG(gravity_x);
    LOG_(">grav_y:"); LOG(gravity_y);
    LOG_(">grav_z:"); LOG(gravity_z);

    float roll = filter.getRoll();
    float pitch = filter.getPitch();

    LOG_(">o_roll:");  LOG(roll);
    LOG_(">o_pitch:"); LOG(pitch);
    
    float ax_ng = ax - gravity_x;
    float ay_ng = ay - gravity_y;
    float az_ng = az - gravity_z;

    LOG_(">ax_ng:"); LOG(ax_ng);
    LOG_(">ay_ng:"); LOG(ay_ng);
    LOG_(">az_ng:"); LOG(az_ng);

    static const float DECAY = 0.975;
    static const float GRAVITY = 9.81;
    static float vx = 0, vy = 0, vz = 0;
    static float px = 0, py = 0, pz = 0;
    float dt = 1.f / SR;

    vx += ax_ng * GRAVITY * dt;
    vy += ay_ng * GRAVITY * dt;
    vz += az_ng * GRAVITY * dt;

    vx *= DECAY;
    vy *= DECAY;
    vz *= DECAY;

    px += vx * dt;
    py += vy * dt;
    pz += vz * dt;

    px *= DECAY;
    py *= DECAY;
    pz *= DECAY;

    LOG_(">vx:"); LOG(vx);
    LOG_(">vy:"); LOG(vy);
    LOG_(">vz:"); LOG(vz);

    LOG_(">px:"); LOG(px);
    LOG_(">py:"); LOG(py);
    LOG_(">pz:"); LOG(pz);

    float a_mag = sqrt(ax_ng*ax_ng);
    // float a_mag = sqrt(ax_ng*ax_ng + ay_ng*ay_ng + az_ng*az_ng);
    strength *= 0.99;
    strength += a_mag / 150.f;
    strength = constrain(strength, 0, 1);

    LOG_(">s:"); LOG(strength);
}
