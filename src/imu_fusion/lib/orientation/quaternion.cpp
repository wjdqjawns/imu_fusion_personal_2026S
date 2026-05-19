#include "quaternion.h"
#include <math.h>

Quat quatIdentity() { return {1.0f, 0.0f, 0.0f, 0.0f}; }

float quatNorm(const Quat& q)
{
    return sqrtf(q.w*q.w + q.x*q.x + q.y*q.y + q.z*q.z);
}

Quat quatNormalize(const Quat& q)
{
    float n = quatNorm(q);
    if (n < 1e-6f) return quatIdentity();
    float inv = 1.0f / n;
    return {q.w*inv, q.x*inv, q.y*inv, q.z*inv};
}

Quat quatConj(const Quat& q) { return {q.w, -q.x, -q.y, -q.z}; }

// Hamilton product; p \kron q
Quat quatMul(const Quat& p, const Quat& q)
{
    return {
        p.w*q.w - p.x*q.x - p.y*q.y - p.z*q.z,
        p.w*q.x + p.x*q.w + p.y*q.z - p.z*q.y,
        p.w*q.y - p.x*q.z + p.y*q.w + p.z*q.x,
        p.w*q.z + p.x*q.y - p.y*q.x + p.z*q.w
    };
}

Quat quatIntegrate(const Quat& q, float gx, float gy, float gz, float dt)
{
    // q_dot = 0.5 * q * omega_pure  where omega_pure = {0, gx, gy, gz}
    Quat omega = {0.0f, gx, gy, gz};
    Quat qdot  = quatMul(q, omega);
    Quat q_new = {
        q.w + 0.5f * qdot.w * dt,
        q.x + 0.5f * qdot.x * dt,
        q.y + 0.5f * qdot.y * dt,
        q.z + 0.5f * qdot.z * dt
    };
    return quatNormalize(q_new);
}
