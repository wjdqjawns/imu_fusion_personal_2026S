#include "madgwick.h"
#include "quaternion.h"
#include "transform.h"
#include <math.h>

void madgwickInit(sMadgwickState& s)
{
    s.q = quatIdentity();
}

void madgwickUpdate(sMadgwickState& s, float ax, float ay, float az, float gx, float gy, float gz, float dt, float beta)
{
    float qw = s.q.w, qx = s.q.x, qy = s.q.y, qz = s.q.z;

    // ── Gyro-only quaternion derivative ──────────────────────────────────
    // q_dot_omega = 0.5 * q ⊗ {0, gx, gy, gz}
    float qdw = 0.5f * (-qx*gx - qy*gy - qz*gz);
    float qdx = 0.5f * ( qw*gx + qy*gz - qz*gy);
    float qdy = 0.5f * ( qw*gy - qx*gz + qz*gx);
    float qdz = 0.5f * ( qw*gz + qx*gy - qy*gx);

    // ── Gradient correction (accel only) ─────────────────────────────────
    float acc_norm = sqrtf(ax*ax + ay*ay + az*az);

    if (acc_norm > 1e-6f)
    {
        ax /= acc_norm; ay /= acc_norm; az /= acc_norm;

        // Objective: f = [2(qx*qz - qw*qy) - ax,
        //                 2(qw*qx + qy*qz) - ay,
        //                 2(0.5 - qx² - qy²) - az]
        float f1 = 2.0f*(qx*qz - qw*qy) - ax;
        float f2 = 2.0f*(qw*qx + qy*qz) - ay;
        float f3 = 1.0f - 2.0f*(qx*qx + qy*qy) - az;

        // Gradient = J^T * f  (J = ∂f/∂q, 3x4)
        // ∂f1/∂[qw,qx,qy,qz] = [-2qy,  2qz, -2qw,  2qx]
        // ∂f2/∂[qw,qx,qy,qz] = [ 2qx,  2qw,  2qz,  2qy]
        // ∂f3/∂[qw,qx,qy,qz] = [   0, -4qx, -4qy,    0]
        float gw = -2.0f*qy*f1 + 2.0f*qx*f2;
        float gx_ =  2.0f*qz*f1 + 2.0f*qw*f2 - 4.0f*qx*f3;
        float gy_ = -2.0f*qw*f1 + 2.0f*qz*f2 - 4.0f*qy*f3;
        float gz_ =  2.0f*qx*f1 + 2.0f*qy*f2;

        float gnorm = sqrtf(gw*gw + gx_*gx_ + gy_*gy_ + gz_*gz_);
        if (gnorm > 1e-6f)
        {
            float inv = beta / gnorm;
            qdw -= gw  * inv;
            qdx -= gx_ * inv;
            qdy -= gy_ * inv;
            qdz -= gz_ * inv;
        }
    }

    // ── Integrate and normalize ───────────────────────────────────────────
    s.q.w = qw + qdw * dt;
    s.q.x = qx + qdx * dt;
    s.q.y = qy + qdy * dt;
    s.q.z = qz + qdz * dt;
    s.q = quatNormalize(s.q);
}

EulerAngle madgwickGetEuler(const sMadgwickState& s)
{
    return quat2euler(s.q);
}