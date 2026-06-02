#pragma once
#include <math.h>

// ─── 6×6 operations ──────────────────────────────────────────────────────────
// C aliases: C must not alias A or B in mul operations

static inline void mat6_zero(float A[6][6])
{
    for (int i = 0; i < 6; i++)
        for (int j = 0; j < 6; j++) A[i][j] = 0.0f;
}

static inline void mat6_identity(float A[6][6])
{
    for (int i = 0; i < 6; i++)
        for (int j = 0; j < 6; j++) A[i][j] = (i == j) ? 1.0f : 0.0f;
}

static inline void mat6_copy(float dst[6][6], const float src[6][6])
{
    for (int i = 0; i < 6; i++)
        for (int j = 0; j < 6; j++) dst[i][j] = src[i][j];
}

static inline void mat6_add(float C[6][6], const float A[6][6], const float B[6][6])
{
    for (int i = 0; i < 6; i++)
        for (int j = 0; j < 6; j++) C[i][j] = A[i][j] + B[i][j];
}

// C = A * B  (C must not alias A or B)
static inline void mat6_mul(float C[6][6], const float A[6][6], const float B[6][6])
{
    for (int i = 0; i < 6; i++)
        for (int j = 0; j < 6; j++)
        {
            float s = 0.0f;
            for (int k = 0; k < 6; k++) s += A[i][k] * B[k][j];
            C[i][j] = s;
        }
}

static inline void mat6_transpose(float AT[6][6], const float A[6][6])
{
    for (int i = 0; i < 6; i++)
        for (int j = 0; j < 6; j++) AT[i][j] = A[j][i];
}

// C = A * B^T  (C must not alias A or B)
static inline void mat6_mul_mat6T(float C[6][6], const float A[6][6], const float B[6][6])
{
    for (int i = 0; i < 6; i++)
        for (int j = 0; j < 6; j++)
        {
            float s = 0.0f;
            for (int k = 0; k < 6; k++) s += A[i][k] * B[j][k];
            C[i][j] = s;
        }
}

// ─── 3×3 operations ──────────────────────────────────────────────────────────

static inline void mat3_zero(float A[3][3])
{
    for (int i = 0; i < 3; i++)
        for (int j = 0; j < 3; j++) A[i][j] = 0.0f;
}

static inline void mat3_identity(float A[3][3])
{
    for (int i = 0; i < 3; i++)
        for (int j = 0; j < 3; j++) A[i][j] = (i == j) ? 1.0f : 0.0f;
}

// ─── Mixed 3×6 / 6×3 / 3×3 operations (Kalman gain) ─────────────────────────

// S(3×3) = H(3×6) * P(6×6) * H^T + r_noise * I
static inline void mat3x6_mul_mat6T_plus_R(float S[3][3],
    const float H[3][6], const float P[6][6], float r_noise)
{
    float tmp[3][6];
    for (int i = 0; i < 3; i++)
        for (int j = 0; j < 6; j++)
        {
            float s = 0.0f;
            for (int k = 0; k < 6; k++) s += H[i][k] * P[k][j];
            tmp[i][j] = s;
        }
    for (int i = 0; i < 3; i++)
        for (int j = 0; j < 3; j++)
        {
            float s = 0.0f;
            for (int k = 0; k < 6; k++) s += tmp[i][k] * H[j][k];
            S[i][j] = s + (i == j ? r_noise : 0.0f);
        }
}

// 3×3 inverse via adjugate; returns false if singular
static inline bool mat3_inv(float inv[3][3], const float A[3][3])
{
    float det = A[0][0] * (A[1][1]*A[2][2] - A[1][2]*A[2][1])
              - A[0][1] * (A[1][0]*A[2][2] - A[1][2]*A[2][0])
              + A[0][2] * (A[1][0]*A[2][1] - A[1][1]*A[2][0]);
    if (fabsf(det) < 1e-10f) return false;
    float id = 1.0f / det;
    inv[0][0] =  (A[1][1]*A[2][2] - A[1][2]*A[2][1]) * id;
    inv[0][1] = -(A[0][1]*A[2][2] - A[0][2]*A[2][1]) * id;
    inv[0][2] =  (A[0][1]*A[1][2] - A[0][2]*A[1][1]) * id;
    inv[1][0] = -(A[1][0]*A[2][2] - A[1][2]*A[2][0]) * id;
    inv[1][1] =  (A[0][0]*A[2][2] - A[0][2]*A[2][0]) * id;
    inv[1][2] = -(A[0][0]*A[1][2] - A[0][2]*A[1][0]) * id;
    inv[2][0] =  (A[1][0]*A[2][1] - A[1][1]*A[2][0]) * id;
    inv[2][1] = -(A[0][0]*A[2][1] - A[0][1]*A[2][0]) * id;
    inv[2][2] =  (A[0][0]*A[1][1] - A[0][1]*A[1][0]) * id;
    return true;
}

// K(6×3) = P(6×6) * H^T(H stored as [3][6]) * Sinv(3×3)
static inline void mat6x3_mul_mat3(float K[6][3], const float P[6][6], const float H[3][6], const float Sinv[3][3])
{
    // PHt(6×3) = P * H^T  (H^T[k][j] = H[j][k])
    float PHt[6][3];
    for (int i = 0; i < 6; i++)
        for (int j = 0; j < 3; j++)
        {
            float s = 0.0f;
            for (int k = 0; k < 6; k++) s += P[i][k] * H[j][k];
            PHt[i][j] = s;
        }
    // K = PHt * Sinv
    for (int i = 0; i < 6; i++)
        for (int j = 0; j < 3; j++)
        {
            float s = 0.0f;
            for (int k = 0; k < 3; k++) s += PHt[i][k] * Sinv[k][j];
            K[i][j] = s;
        }
}
