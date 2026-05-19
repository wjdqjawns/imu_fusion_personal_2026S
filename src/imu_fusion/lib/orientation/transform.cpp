#include "transform.h"
#include "quaternion.h"
#include <math.h>

// ─── Euler ↔ Quaternion ───────────────────────────────────────────────────

Quat euler2quat(const EulerAngle& e)
{
    float hp = 0.5f * e.phi;
    float ht = 0.5f * e.theta;
    float hy = 0.5f * e.psi;

    float cp = cosf(hp), sp = sinf(hp);
    float ct = cosf(ht), st = sinf(ht);
    float cy = cosf(hy), sy = sinf(hy);

    // ZYX: q = q_z * q_y * q_x
    return {
        cy*ct*cp + sy*st*sp,
        cy*ct*sp - sy*st*cp,
        cy*st*cp + sy*ct*sp,
        sy*ct*cp - cy*st*sp
    };
}

EulerAngle quat2euler(const Quat& q)
{
    EulerAngle e;

    // Roll (phi)
    float sinr = 2.0f * (q.w*q.x + q.y*q.z);
    float cosr = 1.0f - 2.0f * (q.x*q.x + q.y*q.y);
    e.phi = atan2f(sinr, cosr);

    // Pitch (theta) — clamped to avoid NaN at ±90°
    float sinp = 2.0f * (q.w*q.y - q.z*q.x);
    e.theta = (fabsf(sinp) >= 1.0f)
                ? copysignf(PI_F * 0.5f, sinp)
                : asinf(sinp);

    // Yaw (psi)
    float siny = 2.0f * (q.w*q.z + q.x*q.y);
    float cosy = 1.0f - 2.0f * (q.y*q.y + q.z*q.z);
    e.psi = atan2f(siny, cosy);

    return e;
}

// ─── Euler ↔ DCM ─────────────────────────────────────────────────────────

Dcm euler2dcm(const EulerAngle& e)
{
    float cp = cosf(e.phi),   sp = sinf(e.phi);
    float ct = cosf(e.theta), st = sinf(e.theta);
    float cy = cosf(e.psi),   sy = sinf(e.psi);

    // R = Rz(psi) * Ry(theta) * Rx(phi)
    Dcm R;
    R.m[0] =  cy*ct;              R.m[1] = cy*st*sp - sy*cp;  R.m[2] = cy*st*cp + sy*sp;
    R.m[3] =  sy*ct;              R.m[4] = sy*st*sp + cy*cp;  R.m[5] = sy*st*cp - cy*sp;
    R.m[6] = -st;                 R.m[7] = ct*sp;             R.m[8] = ct*cp;
    return R;
}

EulerAngle dcm2euler(const Dcm& R)
{
    EulerAngle e;
    e.theta = asinf(-R.m[6]);
    e.phi   = atan2f(R.m[7], R.m[8]);
    e.psi   = atan2f(R.m[3], R.m[0]);
    return e;
}

// ─── DCM ↔ Quaternion ────────────────────────────────────────────────────

Dcm quat2dcm(const Quat& q)
{
    float ww=q.w*q.w, xx=q.x*q.x, yy=q.y*q.y, zz=q.z*q.z;
    float wx=q.w*q.x, wy=q.w*q.y, wz=q.w*q.z;
    float xy=q.x*q.y, xz=q.x*q.z, yz=q.y*q.z;

    Dcm R;
    R.m[0]=ww+xx-yy-zz; R.m[1]=2*(xy-wz);   R.m[2]=2*(xz+wy);
    R.m[3]=2*(xy+wz);   R.m[4]=ww-xx+yy-zz; R.m[5]=2*(yz-wx);
    R.m[6]=2*(xz-wy);   R.m[7]=2*(yz+wx);   R.m[8]=ww-xx-yy+zz;
    return R;
}

// Shepperd's method: picks branch based on largest diagonal element
Quat dcm2quat(const Dcm& R)
{
    const float* m = R.m;
    float trace = m[0] + m[4] + m[8];
    Quat q;

    if (trace > 0.0f) {
        float s = 0.5f / sqrtf(trace + 1.0f);
        q.w = 0.25f / s;
        q.x = (m[7] - m[5]) * s;
        q.y = (m[2] - m[6]) * s;
        q.z = (m[3] - m[1]) * s;
    } else if (m[0] > m[4] && m[0] > m[8]) {
        float s = 2.0f * sqrtf(1.0f + m[0] - m[4] - m[8]);
        q.w = (m[7] - m[5]) / s;
        q.x = 0.25f * s;
        q.y = (m[1] + m[3]) / s;
        q.z = (m[2] + m[6]) / s;
    } else if (m[4] > m[8]) {
        float s = 2.0f * sqrtf(1.0f + m[4] - m[0] - m[8]);
        q.w = (m[2] - m[6]) / s;
        q.x = (m[1] + m[3]) / s;
        q.y = 0.25f * s;
        q.z = (m[5] + m[7]) / s;
    } else {
        float s = 2.0f * sqrtf(1.0f + m[8] - m[0] - m[4]);
        q.w = (m[3] - m[1]) / s;
        q.x = (m[2] + m[6]) / s;
        q.y = (m[5] + m[7]) / s;
        q.z = 0.25f * s;
    }
    return quatNormalize(q);
}
