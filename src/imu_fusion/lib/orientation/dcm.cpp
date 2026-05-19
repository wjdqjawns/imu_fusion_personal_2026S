#include "dcm.h"
#include <math.h>

Dcm dcmIdentity()
{
    Dcm R;
    R.m[0]=1.0f; R.m[1]=0.0f; R.m[2]=0.0f;
    R.m[3]=0.0f; R.m[4]=1.0f; R.m[5]=0.0f;
    R.m[6]=0.0f; R.m[7]=0.0f; R.m[8]=1.0f;
    return R;
}

Dcm dcmIntegrate(const Dcm& R, float gx, float gy, float gz, float dt)
{
    // dR = R * [omega_x] * dt
    // [omega_x] = [[0,-gz,gy],[gz,0,-gx],[-gy,gx,0]]
    // R * [omega_x] row-by-row (R rows are r0, r1, r2):
    //   dR[i][0] =  R[i][1]*gz - R[i][2]*gy
    //   dR[i][1] = -R[i][0]*gz + R[i][2]*gx
    //   dR[i][2] =  R[i][0]*gy - R[i][1]*gx
    const float* m = R.m;
    Dcm Rn;
    float* n = Rn.m;

    n[0] = m[0] + ( m[1]*gz - m[2]*gy) * dt;
    n[1] = m[1] + (-m[0]*gz + m[2]*gx) * dt;
    n[2] = m[2] + ( m[0]*gy - m[1]*gx) * dt;

    n[3] = m[3] + ( m[4]*gz - m[5]*gy) * dt;
    n[4] = m[4] + (-m[3]*gz + m[5]*gx) * dt;
    n[5] = m[5] + ( m[3]*gy - m[4]*gx) * dt;

    n[6] = m[6] + ( m[7]*gz - m[8]*gy) * dt;
    n[7] = m[7] + (-m[6]*gz + m[8]*gx) * dt;
    n[8] = m[8] + ( m[6]*gy - m[7]*gx) * dt;

    return Rn;
}

static float dot3(const float* a, const float* b)
{
    return a[0]*b[0] + a[1]*b[1] + a[2]*b[2];
}

static void normalize3(float* v)
{
    float n = sqrtf(v[0]*v[0] + v[1]*v[1] + v[2]*v[2]);
    if (n < 1e-6f) return;
    float inv = 1.0f / n;
    v[0] *= inv; v[1] *= inv; v[2] *= inv;
}

Dcm dcmNormalize(const Dcm& R)
{
    Dcm Rn = R;
    float* r0 = &Rn.m[0];
    float* r1 = &Rn.m[3];
    float* r2 = &Rn.m[6];

    // Step 1: normalize row0
    normalize3(r0);

    // Step 2: make row1 orthogonal to row0, then normalize
    float d = dot3(r0, r1);
    r1[0] -= d * r0[0];
    r1[1] -= d * r0[1];
    r1[2] -= d * r0[2];
    normalize3(r1);

    // Step 3: row2 = row0 × row1 (already unit length)
    r2[0] = r0[1]*r1[2] - r0[2]*r1[1];
    r2[1] = r0[2]*r1[0] - r0[0]*r1[2];
    r2[2] = r0[0]*r1[1] - r0[1]*r1[0];

    return Rn;
}
