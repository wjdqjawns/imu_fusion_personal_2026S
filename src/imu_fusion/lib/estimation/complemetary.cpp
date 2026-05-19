#include "complementary.h"
#include "euler.h"

void cfInit(CfState& s, float phi0, float theta0, float psi0)
{
    s.phi   = phi0;
    s.theta = theta0;
    s.psi   = psi0;
}

void cfUpdate(CfState& s,
              float gx, float gy, float gz,
              float ax, float ay, float az,
              float dt, float alpha)
{
    // Gyro prediction via Euler kinematics (uses current angle for coupling terms)
    float phi_dot, theta_dot, psi_dot;
    eulerKinematics(s.phi, s.theta, gx, gy, gz,
                    phi_dot, theta_dot, psi_dot);

    float phi_gyro   = s.phi   + phi_dot   * dt;
    float theta_gyro = s.theta + theta_dot * dt;
    float psi_gyro   = s.psi   + psi_dot   * dt;

    // Reference from accelerometer
    float phi_acc, theta_acc;
    accelRollPitch(ax, ay, az, phi_acc, theta_acc);

    // Blend: trust gyro for high-freq, accel for DC
    s.phi   = alpha * phi_gyro   + (1.0f - alpha) * phi_acc;
    s.theta = alpha * theta_gyro + (1.0f - alpha) * theta_acc;
    s.psi   = psi_gyro;  // uncorrected; see cfUpdateYaw
}

void cfUpdateYaw(CfState& s,
                 float mx, float my, float mz,
                 float alpha_yaw)
{
    float psi_mag = magYaw(mx, my, mz, s.phi, s.theta);
    s.psi = alpha_yaw * s.psi + (1.0f - alpha_yaw) * psi_mag;
}

EulerAngle cfGetEuler(const CfState& s)
{
    return {s.phi, s.theta, s.psi};
}
