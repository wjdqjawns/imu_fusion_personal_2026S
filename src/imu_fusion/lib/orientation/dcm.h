#pragma once
#include "types.h"

Dcm dcmIdentity();

// First-order Euler integration: dR/dt = R * [omega_x]
// [omega_x] is the skew-symmetric matrix of body angular rates
Dcm dcmIntegrate(const Dcm& R, float gx, float gy, float gz, float dt);

// Gram-Schmidt orthonormalization — call periodically to prevent drift
// Renormalizes row0, re-orthogonalizes row1, recomputes row2 = row0 × row1
Dcm dcmNormalize(const Dcm& R);
