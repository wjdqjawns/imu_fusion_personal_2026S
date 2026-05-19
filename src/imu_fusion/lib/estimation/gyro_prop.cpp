#include "gyro_prop.h"
#include "quaternion.h"
#include "euler.h"
#include "dcm.h"
#include "transform.h"

void gyroPropInit(GyroPropState& s)
{
    s.euler = {0.0f, 0.0f, 0.0f};
    s.quat  = quatIdentity();
    s.dcm   = dcmIdentity();
}

void gyroPropUpdate(GyroPropState& s, float gx, float gy, float gz, float dt)
{
    // Euler kinematics: integrate angle rates from kinematic equations
    float phi_dot, theta_dot, psi_dot;
    eulerKinematics(s.euler.phi, s.euler.theta, gx, gy, gz, phi_dot, theta_dot, psi_dot);
    s.euler.phi   += phi_dot   * dt;
    s.euler.theta += theta_dot * dt;
    s.euler.psi   += psi_dot   * dt;

    // Quaternion kinematics: q_dot = 0.5 * q * omega
    s.quat = quatIntegrate(s.quat, gx, gy, gz, dt);

    // DCM kinematics: dR/dt = R * [omega_x]
    s.dcm = dcmIntegrate(s.dcm, gx, gy, gz, dt);

    // Periodic re-orthonormalization to prevent DCM drift
    // (every step is fine; cheap enough on AVR for 100 Hz)
    s.dcm = dcmNormalize(s.dcm);
}

EulerAngle gyroPropGetEulerFromEuler(const GyroPropState& s) { return s.euler; }
EulerAngle gyroPropGetEulerFromQuat(const GyroPropState& s)  { return quat2euler(s.quat); }
EulerAngle gyroPropGetEulerFromDcm(const GyroPropState& s)   { return dcm2euler(s.dcm); }