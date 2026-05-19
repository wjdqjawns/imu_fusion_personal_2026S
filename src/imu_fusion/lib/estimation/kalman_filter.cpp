#include "kalman_filter.h"
#include "euler.h"

static void axisInit(EkfAxis& a, float angle0, float q_ang, float q_bi, float r_ang)
{
    a.angle   = angle0;
    a.bias    = 0.0f;
    a.P[0][0] = 1.0f; a.P[0][1] = 0.0f;
    a.P[1][0] = 0.0f; a.P[1][1] = 1.0f;
    a.q_angle = q_ang;
    a.q_bias  = q_bi;
    a.r_angle = r_ang;
}

// Single-axis predict + update
// rate  : raw body rate driving this axis [rad/s]
// meas  : measurement from accelerometer  [rad]
// dt    : time step [s]
static void axisUpdate(EkfAxis& a, float rate, float meas, float dt)
{
    // ── Predict ──────────────────────────────────────────────────────────
    float angle_pred = a.angle + (rate - a.bias) * dt;

    // P_pred = A*P*A^T + Q,  A = [[1,-dt],[0,1]]
    float P00 = a.P[0][0] + dt*(dt*a.P[1][1] - a.P[0][1] - a.P[1][0]) + a.q_angle;
    float P01 = a.P[0][1] - dt * a.P[1][1];
    float P10 = a.P[1][0] - dt * a.P[1][1];
    float P11 = a.P[1][1] + a.q_bias;

    // ── Update (H = [1, 0]) ───────────────────────────────────────────────
    float S   = P00 + a.r_angle;   // innovation variance
    float K0  = P00 / S;           // Kalman gain for angle
    float K1  = P10 / S;           // Kalman gain for bias

    float innov = meas - angle_pred;
    a.angle = angle_pred + K0 * innov;
    a.bias  = a.bias     + K1 * innov;

    // P_new = (I - K*H) * P_pred
    a.P[0][0] = (1.0f - K0) * P00;
    a.P[0][1] = (1.0f - K0) * P01;
    a.P[1][0] = P10 - K1 * P00;
    a.P[1][1] = P11 - K1 * P01;
}

void ekfInit(EkfState& s, float phi0, float theta0)
{
    axisInit(s.phi,   phi0,   EKF_Q_ANGLE, EKF_Q_BIAS, EKF_R_ANGLE);
    axisInit(s.theta, theta0, EKF_Q_ANGLE, EKF_Q_BIAS, EKF_R_ANGLE);
}

void ekfUpdate(EkfState& s, float gx, float gy, float gz,
               float ax, float ay, float az, float dt)
{
    // Reference angles from accelerometer (static assumption)
    float phi_acc, theta_acc;
    accelRollPitch(ax, ay, az, phi_acc, theta_acc);

    // Euler kinematics with current bias estimate for more accurate prediction
    // (use current angle estimate + bias-corrected rates)
    float phi_dot, theta_dot, psi_dot;
    eulerKinematics(s.phi.angle, s.theta.angle,
                    gx - s.phi.bias,
                    gy - s.theta.bias,
                    gz,
                    phi_dot, theta_dot, psi_dot);

    // Feed Euler rates as effective body rates for each axis.
    // axisUpdate predicts: angle += (rate - bias)*dt
    // We pre-subtract bias above and pass gx/gy here so the model stays valid.
    axisUpdate(s.phi,   gx, phi_acc,   dt);
    axisUpdate(s.theta, gy, theta_acc, dt);

    (void)gz; (void)phi_dot; (void)theta_dot; (void)psi_dot;
}

EulerAngle ekfGetEuler(const EkfState& s)
{
    return {s.phi.angle, s.theta.angle, 0.0f};
}
