#include "euler.h"
#include <math.h>

void eulerKinematics(float phi, float theta,
                     float gx, float gy, float gz,
                     float& phi_dot, float& theta_dot, float& psi_dot)
{
    float sp = sinf(phi), cp = cosf(phi);
    float tt = tanf(theta), ct = cosf(theta);

    phi_dot   = gx + (gy * sp + gz * cp) * tt;
    theta_dot = gy * cp - gz * sp;

    // avoid gimbal lock at theta = \pm \pi / 2
    psi_dot   = (fabsf(ct) < 1e-4f) ? 0.0f : (gy * sp + gz * cp) / ct;
}

void accelRollPitch(float ax, float ay, float az, float& phi, float& theta)
{
    // NED frame: body at rest → az ≈ +1g (gravity along +Z body axis)
    // phi   = atan2(ay, az)      roll  from Y vs Z gravity components
    // theta = atan2(-ax, √(ay²+az²))  pitch from X vs lateral
    phi   = atan2f(ay, az);
    theta = atan2f(-ax, sqrtf(ay*ay + az*az));
}

float magYaw(float mx, float my, float mz, float phi, float theta)
{
    float sp = sinf(phi),  cp = cosf(phi);
    float st = sinf(theta), ct = cosf(theta);

    // Tilt-compensate: rotate magnetometer to horizontal plane
    float mx_h =  mx * ct + mz * st;
    float my_h =  mx * st * sp - my * cp + mz * ct * sp;

    return atan2f(-my_h, mx_h);
}
