#include "kalman_filter.h"

void KalmanFilter::init(float phi0, float theta0) {
    _phi     = phi0;
    _theta   = theta0;
    _P_phi   = 1.0f;
    _P_theta = 1.0f;
}

void KalmanFilter::update(const ImuData& d, float Ts) {
    // prediction
    float phi_dot, theta_dot;
    gyroRates(d, _phi, _theta, phi_dot, theta_dot);

    float phi_pred   = _phi   + Ts * phi_dot;
    float theta_pred = _theta + Ts * theta_dot;
    float P_phi_p    = _P_phi   + Q_phi;
    float P_theta_p  = _P_theta + Q_theta;

    // measurement update
    float phi_acc, theta_acc;
    accelAngles(d, phi_acc, theta_acc);

    float K_phi   = P_phi_p   / (P_phi_p   + R_phi);
    float K_theta = P_theta_p / (P_theta_p + R_theta);

    _phi    = phi_pred   + K_phi   * (phi_acc   - phi_pred);
    _theta  = theta_pred + K_theta * (theta_acc - theta_pred);
    _P_phi   = (1.0f - K_phi)   * P_phi_p;
    _P_theta = (1.0f - K_theta) * P_theta_p;
}