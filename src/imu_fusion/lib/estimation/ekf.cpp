#include "ekf.h"
#include "euler.h"
#include "dcm.h"
#include "mat6.h"
#include <math.h>

static float norm3(float a, float b, float c)
{
    return sqrtf(a*a + b*b + c*c);
}

// S = H*P*H^T + r*I,  K = P*H^T*S^-1,  x += K*(z-h),  P = (I-K*H)*P
// All large buffers are static to avoid stack overflow on AVR.
static void ekfKalmanUpdate(EkfState& s,
    const float H[3][6], const float z[3], const float h[3], float r_noise)
{
    static float S[3][3], Sinv[3][3], K[6][3], KH[6][6], Pnew[6][6];

    mat3x6_mul_mat6T_plus_R(S, H, s.P, r_noise);
    if (!mat3_inv(Sinv, S)) return;
    mat6x3_mul_mat3(K, s.P, H, Sinv);

    // x += K * (z - h)
    for (int i = 0; i < 6; i++) {
        float upd = 0.0f;
        for (int j = 0; j < 3; j++) upd += K[i][j] * (z[j] - h[j]);
        s.x[i] += upd;
    }
    s.x[2] = wrapAngle(s.x[2]);

    // KH = K * H  (6×3 × 3×6 = 6×6)
    for (int i = 0; i < 6; i++)
        for (int j = 0; j < 6; j++) {
            float v = 0.0f;
            for (int k = 0; k < 3; k++) v += K[i][k] * H[k][j];
            KH[i][j] = v;
        }

    // Pnew = (I - KH) * P
    for (int i = 0; i < 6; i++)
        for (int j = 0; j < 6; j++) {
            float v = 0.0f;
            for (int k = 0; k < 6; k++) {
                float ikh = (i == k ? 1.0f : 0.0f) - KH[i][k];
                v += ikh * s.P[k][j];
            }
            Pnew[i][j] = v;
        }
    mat6_copy(s.P, Pnew);
}

// ─── public API ───────────────────────────────────────────────────────────────

void ekfInit(EkfState& s, float phi0, float theta0, float psi0,
             float mx_ref, float my_ref, float mz_ref)
{
    s.x[0] = phi0;  s.x[1] = theta0; s.x[2] = psi0;
    s.x[3] = 0.0f;  s.x[4] = 0.0f;  s.x[5] = 0.0f;
    mat6_identity(s.P);
    s.mx_ref = mx_ref;
    s.my_ref = my_ref;
    s.mz_ref = mz_ref;
}

void ekfPredict(EkfState& s, float gx_rad, float gy_rad, float gz_rad, float dt)
{
    float phi   = s.x[0], theta = s.x[1];
    float sp    = sinf(phi), cp = cosf(phi);
    float st    = sinf(theta), ct = cosf(theta);

    float p = gx_rad - s.x[3];
    float q = gy_rad - s.x[4];
    float r = gz_rad - s.x[5];

    float phi_dot, theta_dot, psi_dot;
    eulerKinematics(phi, theta, p, q, r, phi_dot, theta_dot, psi_dot);

    s.x[0] += phi_dot   * dt;
    s.x[1] += theta_dot * dt;
    s.x[2]  = wrapAngle(s.x[2] + psi_dot * dt);
    // x[3..5] unchanged (random walk)

    // ── F Jacobian ────────────────────────────────────────────────────────────
    static float F[6][6], temp[6][6], Pnew[6][6];
    mat6_identity(F);

    float tt   = (fabsf(ct) > 1e-4f) ? (st / ct) : 0.0f;
    float sec2 = (fabsf(ct) > 1e-4f) ? (1.0f / (ct*ct)) : 0.0f;

    float spq = q*sp + r*cp;    // q·sinφ + r·cosφ
    float cpq = q*cp - r*sp;    // q·cosφ - r·sinφ

    // row 0: ∂phi_new/∂[phi,theta,*,b_gx,b_gy,b_gz]
    F[0][0] += cpq * tt   * dt;
    F[0][1] += spq * sec2 * dt;
    F[0][3]  = -dt;
    F[0][4]  = -sp * tt * dt;
    F[0][5]  = -cp * tt * dt;

    // row 1: ∂theta_new/∂[phi,*,*,*,b_gy,b_gz]
    F[1][0]  = -spq * dt;
    F[1][4]  = -cp  * dt;
    F[1][5]  =  sp  * dt;

    // row 2: ∂psi_new/∂[phi,theta,*,*,b_gy,b_gz]
    if (fabsf(ct) > 1e-4f) {
        F[2][0]  =  cpq / ct * dt;
        F[2][1]  =  spq * st / (ct*ct) * dt;
        F[2][4]  = -sp  / ct * dt;
        F[2][5]  = -cp  / ct * dt;
    }
    // rows 3-5 stay as identity

    // ── P = F*P*F^T + Q ───────────────────────────────────────────────────────
    mat6_mul(temp, F, s.P);
    mat6_mul_mat6T(Pnew, temp, F);
    for (int i = 0; i < 6; i++)
        Pnew[i][i] += (i < 3) ? EKF_Q_ANGLE : EKF_Q_BIAS;
    mat6_copy(s.P, Pnew);
}

void ekfUpdateAccel(EkfState& s, float ax, float ay, float az)
{
    if (fabsf(norm3(ax, ay, az) * 9.81f - 9.81f) > ACCEL_THRESH) return;
    if (fabsf(s.x[1]) > 85.0f * 0.01745329f) return;

    float phi = s.x[0], theta = s.x[1];
    float sp = sinf(phi), cp = cosf(phi);
    float st = sinf(theta), ct = cosf(theta);

    // predicted gravity in body frame [m/s²]
    float h[3] = {
        -st       * 9.81f,
         ct * sp  * 9.81f,
         ct * cp  * 9.81f
    };
    float z[3] = { ax * 9.81f, ay * 9.81f, az * 9.81f };

    static float H[3][6];
    for (int i = 0; i < 3; i++)
        for (int j = 0; j < 6; j++) H[i][j] = 0.0f;

    H[0][1] = -ct       * 9.81f;
    H[1][0] =  ct * cp  * 9.81f;
    H[1][1] = -st * sp  * 9.81f;
    H[2][0] = -ct * sp  * 9.81f;
    H[2][1] = -st * cp  * 9.81f;

    ekfKalmanUpdate(s, H, z, h, EKF_R_ACCEL);
}

void ekfUpdateMag(EkfState& s, float mx, float my, float mz)
{
    float mag_norm = norm3(mx, my, mz);
    float ref_norm = norm3(s.mx_ref, s.my_ref, s.mz_ref);
    if (fabsf(mag_norm - ref_norm) > MAG_THRESH) return;
    if (fabsf(s.x[1]) > 85.0f * 0.01745329f) return;

    float phi = s.x[0], theta = s.x[1], psi = s.x[2];
    float sp = sinf(phi), cp = cosf(phi);
    float st = sinf(theta), ct = cosf(theta);
    float sy = sinf(psi),   cy = cosf(psi);

    // predicted mag in body frame: h = R_bw * ref
    float R_wb[9], R_bw[9];
    dcmFromEuler(phi, theta, psi, R_wb);
    dcmTranspose(R_wb, R_bw);

    float ref[3] = { s.mx_ref, s.my_ref, s.mz_ref };
    float h[3];
    dcmRotate(R_bw, ref, h);

    float z[3] = { mx, my, mz };

    // H_m (3×6): analytical ∂h/∂[phi,theta,psi,0,0,0]
    float rx = s.mx_ref, ry = s.my_ref, rz = s.mz_ref;
    static float H[3][6];
    for (int i = 0; i < 3; i++)
        for (int j = 0; j < 6; j++) H[i][j] = 0.0f;

    H[0][1] = -cy*st*rx - sy*st*ry - ct*rz;
    H[0][2] = -sy*ct*rx + cy*ct*ry;

    H[1][0] =  (cy*st*cp + sy*sp)*rx + (sy*st*cp - cy*sp)*ry + ct*cp*rz;
    H[1][1] =   cy*ct*sp*rx + sy*ct*sp*ry - st*sp*rz;
    H[1][2] =  (-sy*st*sp - cy*cp)*rx + (cy*st*sp - sy*cp)*ry;

    H[2][0] =  (-cy*st*sp + sy*cp)*rx + (-sy*st*sp - cy*cp)*ry - ct*sp*rz;
    H[2][1] =   cy*ct*cp*rx + sy*ct*cp*ry - st*cp*rz;
    H[2][2] =  (-sy*st*cp + cy*sp)*rx + (cy*st*cp + sy*sp)*ry;

    ekfKalmanUpdate(s, H, z, h, EKF_R_MAG);
}

EulerAngle ekfGetEuler(const EkfState& s)
{
    return { s.x[0], s.x[1], s.x[2] };
}
