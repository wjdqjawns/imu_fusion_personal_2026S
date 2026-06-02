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

// ─── Raw float[9] variants ────────────────────────────────────────────────────

void dcmFromEuler(float phi, float theta, float psi, float R[9])
{
    float sp = sinf(phi), cp = cosf(phi);
    float st = sinf(theta), ct = cosf(theta);
    float sy = sinf(psi),  cy = cosf(psi);

    // R_body2world for ZYX convention: Rz(psi)*Ry(theta)*Rx(phi), row-major
    R[0] = cy*ct;               R[1] = cy*st*sp - sy*cp;    R[2] = cy*st*cp + sy*sp;
    R[3] = sy*ct;               R[4] = sy*st*sp + cy*cp;    R[5] = sy*st*cp - cy*sp;
    R[6] = -st;                 R[7] = ct*sp;               R[8] = ct*cp;
}

void dcmTranspose(const float R[9], float RT[9])
{
    RT[0]=R[0]; RT[1]=R[3]; RT[2]=R[6];
    RT[3]=R[1]; RT[4]=R[4]; RT[5]=R[7];
    RT[6]=R[2]; RT[7]=R[5]; RT[8]=R[8];
}

void dcmRotate(const float R[9], const float x[3], float y[3])
{
    y[0] = R[0]*x[0] + R[1]*x[1] + R[2]*x[2];
    y[1] = R[3]*x[0] + R[4]*x[1] + R[5]*x[2];
    y[2] = R[6]*x[0] + R[7]*x[1] + R[8]*x[2];
}
