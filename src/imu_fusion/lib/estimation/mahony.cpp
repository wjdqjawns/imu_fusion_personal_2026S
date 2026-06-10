#include "mahony.h"
#include "quaternion.h"
#include "transform.h"
#include <math.h>

void mahonyInit(MahonyState& s)
{
    s.q      = quatIdentity();
    s.ex_int = 0.0f;
    s.ey_int = 0.0f;
    s.ez_int = 0.0f;
}

void mahonyUpdate(MahonyState& s, float ax, float ay, float az, float gx, float gy, float gz, float dt, float Kp, float Ki)
{
    // Skip accelerometer correction if magnitude is unreliable
    float acc_norm = sqrtf(ax*ax + ay*ay + az*az);
    if (acc_norm < 1e-6f)
    {
        s.q = quatIntegrate(s.q, gx, gy, gz, dt);
        return;
    }
    ax /= acc_norm; ay /= acc_norm; az /= acc_norm;

    // Estimated gravity direction in body frame derived from quaternion
    // (third column of R^T where R is body-to-world rotation)
    float vx = 2.0f*(s.q.x*s.q.z - s.q.w*s.q.y);
    float vy = 2.0f*(s.q.w*s.q.x + s.q.y*s.q.z);
    float vz = s.q.w*s.q.w - s.q.x*s.q.x - s.q.y*s.q.y + s.q.z*s.q.z;

    // Error: cross product of measured vs estimated gravity
    // (rotation needed to align v with a)
    float ex = ay*vz - az*vy;
    float ey = az*vx - ax*vz;
    float ez = ax*vy - ay*vx;

    // PI correction on gyro rates
    s.ex_int += ex * dt;
    s.ey_int += ey * dt;
    s.ez_int += ez * dt;

    float gx_c = gx + Kp*ex + Ki*s.ex_int;
    float gy_c = gy + Kp*ey + Ki*s.ey_int;
    float gz_c = gz + Kp*ez + Ki*s.ez_int;

    s.q = quatIntegrate(s.q, gx_c, gy_c, gz_c, dt);
}

EulerAngle mahonyGetEuler(const MahonyState& s)
{
    return quat2euler(s.q);
}