#pragma once

#include "types.h"
#include "global_var.h"
#include "orientation.h"

class KalmanFilter {
public:
    // 노이즈 파라미터 (globals.h 상수 기본값 사용)
    float Q_phi   = KF_Q_PHI;
    float Q_theta = KF_Q_THETA;
    float R_phi   = KF_R_PHI;
    float R_theta = KF_R_THETA;

    void init(float phi0 = 0.0f, float theta0 = 0.0f);
    void update(const ImuData& d, float Ts);

    EulerAngle getEuler() const { return {_phi, _theta, 0.0f}; }
    float phi()   const { return _phi; }
    float theta() const { return _theta; }

private:
    float _phi   = 0.0f;
    float _theta = 0.0f;
    float _P_phi   = 1.0f;
    float _P_theta = 1.0f;
};