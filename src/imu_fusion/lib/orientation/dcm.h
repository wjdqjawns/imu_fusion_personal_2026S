#pragma once
#include "types.h"

Dcm dcmIdentity();

// First-order Euler integration: dR/dt = R * [omega_x]
// [omega_x] is the skew-symmetric matrix of body angular rates
Dcm dcmIntegrate(const Dcm& R, float gx, float gy, float gz, float dt);

// Gram-Schmidt orthonormalization — call periodically to prevent drift
// Renormalizes row0, re-orthogonalizes row1, recomputes row2 = row0 × row1
Dcm dcmNormalize(const Dcm& R);

// ─── Raw float[9] variants used by EKF ───────────────────────────────────────

// ZYX Euler → R_body2world (row-major float[9])
void dcmFromEuler(float phi, float theta, float psi, float R[9]);

// Transpose: RT = R^T  (R_world2body = R_body2world^T)
void dcmTranspose(const float R[9], float RT[9]);

// Rotate vector: y = R * x  (3×3 row-major)
void dcmRotate(const float R[9], const float x[3], float y[3]);
