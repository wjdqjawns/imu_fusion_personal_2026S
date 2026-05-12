#include "orientation.h"

void accelAngles(const ImuData& d, float& phi_deg, float& theta_deg)
{
    phi_deg   = atan2f(-d.ay, -d.az) * RAD2DEG;
    theta_deg = atan2f( d.ax, sqrtf(d.ay * d.ay + d.az * d.az)) * RAD2DEG;
}

void gyroRates(const ImuData& d, float phi_deg, float theta_deg, float& phi_dot, float& theta_dot)
{
    float phi   = phi_deg   * DEG2RAD;
    float theta = theta_deg * DEG2RAD;
    
    phi_dot   = d.gx + d.gy * sinf(phi) * tanf(theta) + d.gz * cosf(phi) * tanf(theta);
    theta_dot =        d.gy * cosf(phi)                - d.gz * sinf(phi);
}

void euler2rot(const EulerAngle& e, float R[3][3])
{
    float cp = cosf(e.phi   * DEG2RAD), sp = sinf(e.phi   * DEG2RAD);
    float ct = cosf(e.theta * DEG2RAD), st = sinf(e.theta * DEG2RAD);
    float cy = cosf(e.psi   * DEG2RAD), sy = sinf(e.psi   * DEG2RAD);
    
    // R = Rz * Ry * Rx
    R[0][0] = cy*ct;  R[0][1] = cy*st*sp - sy*cp;  R[0][2] = cy*st*cp + sy*sp;
    R[1][0] = sy*ct;  R[1][1] = sy*st*sp + cy*cp;  R[1][2] = sy*st*cp - cy*sp;
    R[2][0] = -st;    R[2][1] = ct*sp;              R[2][2] = ct*cp;
}

Quaternion rot2quat(const float R[3][3])
{
    Quaternion q;
    float trace = R[0][0] + R[1][1] + R[2][2];

    if (trace > 0.0f)
    {
        float s = 0.5f / sqrtf(trace + 1.0f);
        q.w = 0.25f / s;
        q.x = (R[2][1] - R[1][2]) * s;
        q.y = (R[0][2] - R[2][0]) * s;
        q.z = (R[1][0] - R[0][1]) * s;
    }
    else if (R[0][0] > R[1][1] && R[0][0] > R[2][2])
    {
        float s = 2.0f * sqrtf(1.0f + R[0][0] - R[1][1] - R[2][2]);
        q.w = (R[2][1] - R[1][2]) / s;
        q.x = 0.25f * s;
        q.y = (R[0][1] + R[1][0]) / s;
        q.z = (R[0][2] + R[2][0]) / s;
    }
    else if (R[1][1] > R[2][2])
    {
        float s = 2.0f * sqrtf(1.0f + R[1][1] - R[0][0] - R[2][2]);
        q.w = (R[0][2] - R[2][0]) / s;
        q.x = (R[0][1] + R[1][0]) / s;
        q.y = 0.25f * s;
        q.z = (R[1][2] + R[2][1]) / s;
    }
    else
    {
        float s = 2.0f * sqrtf(1.0f + R[2][2] - R[0][0] - R[1][1]);
        q.w = (R[1][0] - R[0][1]) / s;
        q.x = (R[0][2] + R[2][0]) / s;
        q.y = (R[1][2] + R[2][1]) / s;
        q.z = 0.25f * s;
    }
    return quatNormalize(q);
}

Quaternion euler2quat(const EulerAngle& e)
{
    float R[3][3];
    euler2rot(e, R);
    return rot2quat(R);
}

EulerAngle quat2euler(const Quaternion& q)
{
    EulerAngle e;
    
    // phi (roll)
    float sinr = 2.0f * (q.w * q.x + q.y * q.z);
    float cosr = 1.0f - 2.0f * (q.x * q.x + q.y * q.y);
    e.phi = atan2f(sinr, cosr) * RAD2DEG;
    
    // theta (pitch)
    float sinp = 2.0f * (q.w * q.y - q.z * q.x);
    e.theta = (fabsf(sinp) >= 1.0f) ? copysignf(90.0f, sinp) : asinf(sinp) * RAD2DEG;
    
    // psi (yaw)
    float siny = 2.0f * (q.w * q.z + q.x * q.y);
    float cosy = 1.0f - 2.0f * (q.y * q.y + q.z * q.z);
    e.psi = atan2f(siny, cosy) * RAD2DEG;
    
    return e;
}

Quaternion quatMul(const Quaternion& p, const Quaternion& q)
{
    return
    {
        p.w*q.w - p.x*q.x - p.y*q.y - p.z*q.z,
        p.w*q.x + p.x*q.w + p.y*q.z - p.z*q.y,
        p.w*q.y - p.x*q.z + p.y*q.w + p.z*q.x,
        p.w*q.z + p.x*q.y - p.y*q.x + p.z*q.w
    };
}

Quaternion quatNormalize(const Quaternion& q)
{
    float n = sqrtf(q.w*q.w + q.x*q.x + q.y*q.y + q.z*q.z);
    
    if (n < 1e-6f)
    {
        return {1.0f, 0.0f, 0.0f, 0.0f};
    }

    return {q.w/n, q.x/n, q.y/n, q.z/n};
}