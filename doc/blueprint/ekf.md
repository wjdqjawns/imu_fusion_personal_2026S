## 목표
PlatformIO / Arduino 환경에서 MPU9250 IMU의 roll(φ), pitch(θ), yaw(ψ)를
여러 필터로 동시에 추정하고 Serial CSV로 출력하는 코드를 작성해줘.
이번 요청은 공통 기반 구조 + EKF까지만 구현해줘.

────────────────────────────────────────
## 하드웨어 / 라이브러리 환경
────────────────────────────────────────
- MCU: Arduino Uno
- 센서 라이브러리: MPU9250_asukiaaa
    accelX/Y/Z() → [g]
    gyroX/Y/Z()  → [°/s]  ← 필터 입력 전 DEG2RAD 변환 필요
    magX/Y/Z()   → [μT]
- PlatformIO 프로젝트
- 제약: malloc/new 금지, Eigen 금지, float 배열로만 행렬 연산

────────────────────────────────────────
## Euler 규약
────────────────────────────────────────
ZYX aerospace convention (Tait-Bryan)
  φ = roll  (x축),  θ = pitch (y축),  ψ = yaw (z축)
내부: [rad], Serial 출력: [deg]

────────────────────────────────────────
## 파일 구조
────────────────────────────────────────
include/
  types.h          ← 아래 타입 정의
  global_var.h     ← #define 모드/필터 선택, constexpr 파라미터

lib/orientation/
  euler.h/.cpp     ← accelRollPitch(), magYaw(), eulerKinematics()
  dcm.h/.cpp       ← DCM 유틸리티 (회전행렬 R 생성)
  quaternion.h/.cpp
  transform.h/.cpp

lib/math/
  mat6.h           ← 6×6 행렬 연산 인라인 함수 (곱, 전치, 덧셈, 역행렬 아래 참조)
  mat3.h           ← 3×3 행렬 연산 인라인 함수

lib/imu/
  imu_sensor.h/.cpp
  imu_calibration.h/.cpp

lib/estimation/
  gyro_prop.h/.cpp
  ekf.h/.cpp       ← 이번에 구현
  complementary.h/.cpp
  mahony.h/.cpp
  madgwick.h/.cpp

src/
  global_var.cpp
  main.cpp

────────────────────────────────────────
## types.h 정의
────────────────────────────────────────
typedef struct { float ax,ay,az; float gx,gy,gz; float mx,my,mz; } ImuData;
typedef struct { float ax,ay,az; float gx,gy,gz; float mx,my,mz; } ImuBias;
typedef struct { float phi, theta, psi; } EulerAngle;
typedef struct { float w,x,y,z; } Quat;
typedef struct { float m[9]; } Dcm;   // row-major

────────────────────────────────────────
## global_var.h — 모드 및 파라미터
────────────────────────────────────────

// 실행 모드 (하나만 활성화)
#define MODE_STATE_ESTIMATION
// #define MODE_RAW
// #define MODE_CALIBRATION

// 필터 선택 (복수 가능)
#define FILTER_GYRO_ONLY
#define FILTER_EKF
// #define FILTER_ESKF       ← 다음 프롬프트에서 추가
// #define FILTER_COMPLEMENTARY
// #define FILTER_MAHONY
// #define FILTER_MADGWICK

// 타이밍
constexpr float Ts       = 0.01f;
constexpr int   LOOP_MS  = 10;
constexpr long  SERIAL_BAUD = 115200;

// EKF 튜닝
constexpr float EKF_Q_ANGLE  = 0.001f;
constexpr float EKF_Q_BIAS   = 0.003f;
constexpr float EKF_R_ACCEL  = 0.03f;
constexpr float EKF_R_MAG    = 0.1f;
constexpr float ACCEL_THRESH = 1.5f;   // [m/s²] 동적 구간 판별
constexpr float MAG_THRESH   = 5.0f;   // [μT]   자기장 교란 판별

// CF / Mahony / Madgwick
constexpr float CF_ALPHA      = 0.98f;
constexpr float MAHONY_KP     = 2.0f;
constexpr float MAHONY_KI     = 0.005f;
constexpr float MADGWICK_BETA = 0.1f;

────────────────────────────────────────
## lib/math/mat6.h — 행렬 유틸리티
────────────────────────────────────────
6×6 float 배열 기반. 아래 함수를 inline으로 구현:
  mat6_zero(A)
  mat6_identity(A)
  mat6_copy(dst, src)
  mat6_add(C, A, B)          C = A + B
  mat6_mul(C, A, B)          C = A * B
  mat6_transpose(AT, A)      AT = A^T
  mat6_mul_mat6T(C, A, B)    C = A * B^T

3×6, 6×3, 3×3 혼합 연산 (Kalman gain 계산용):
  mat3_zero(A), mat3_identity(A)
  mat3x6_mul_mat6T_plus_R(S, H, P, R)   S = H*P*H^T + R  (3×3)
  mat3_inv(inv, A)                        3×3 역행렬 (수반행렬 방법)
  mat6x3_mul_mat3(K, P, HT, Sinv)       K = P*H^T*S^-1  (6×3)

────────────────────────────────────────
## lib/orientation/euler.h/.cpp
────────────────────────────────────────

// 정지 상태 accel → roll/pitch 초기값
void accelRollPitch(float ax, float ay, float az,
                    float& phi, float& theta);

// tilt 보정된 mag → yaw 초기값
float magYaw(float mx, float my, float mz,
             float phi, float theta);

// Euler kinematics: body 각속도 → world Euler rate
// 입력: 현재 phi,theta / 바이어스 보정된 p,q,r [rad/s]
// 출력: phi_dot, theta_dot, psi_dot
void eulerKinematics(float phi, float theta,
                     float p, float q, float r,
                     float& phi_dot, float& theta_dot, float& psi_dot);

// [-π, π] 각도 wrap
float wrapAngle(float a);

────────────────────────────────────────
## lib/orientation/dcm.h/.cpp
────────────────────────────────────────

// ZYX Euler → 3×3 DCM R_body2world (row-major)
void dcmFromEuler(float phi, float theta, float psi, float R[9]);

// R_body2world의 전치 = R_world2body
void dcmTranspose(const float R[9], float RT[9]);

// 3×1 벡터 회전: y = R * x
void dcmRotate(const float R[9], const float x[3], float y[3]);

────────────────────────────────────────
## EKF 상세 스펙 (lib/estimation/ekf.h/.cpp)
────────────────────────────────────────

### 상태 벡터 (6×1)
x = [ φ, θ, ψ, b_gx, b_gy, b_gz ]^T
  φ,θ,ψ    : roll/pitch/yaw [rad], world frame
  b_g{xyz} : 자이로 바이어스 [rad/s], random walk 모델

### EkfState 구조체
typedef struct {
    float x[6];       // 상태 벡터
    float P[6][6];    // 오차 공분산
    float mx_ref;     // 초기화 시 저장한 world frame 지자기 레퍼런스
    float my_ref;
    float mz_ref;
} EkfState;

### Predict (매 루프)
1. 바이어스 보정:
   p = gx_rad - x[3]
   q = gy_rad - x[4]
   r = gz_rad - x[5]

2. Euler kinematics로 φ,θ,ψ 전파:
   eulerKinematics(x[0], x[1], p, q, r, phi_dot, theta_dot, psi_dot)
   x[0] += phi_dot   * dt
   x[1] += theta_dot * dt
   x[2] += psi_dot   * dt
   x[2]  = wrapAngle(x[2])
   x[3..5] 유지 (random walk)

3. F 자코비안 (6×6), 수동 계산:
   sp=sinφ, cp=cosφ, tt=tanθ, ct2=1/cos²θ
   spq = q·sp + r·cp,  cpq = q·cp - r·sp

   F = I (단위행렬 시작)
   F[0][0] += cpq * tt * dt
   F[0][1] += spq * ct2 * dt
   F[0][3]  = -dt
   F[0][4]  = -sp*tt*dt
   F[0][5]  = -cp*tt*dt

   F[1][0]  = -spq * dt        (= cpq로 부호 주의)
   F[1][4]  = -cp*dt
   F[1][5]  =  sp*dt

   costh = cosf(x[1])
   F[2][0]  = spq/costh * dt
   F[2][1]  = spq*sinf(x[1])/(costh*costh) * dt
   F[2][4]  = -sp/costh * dt
   F[2][5]  = -cp/costh * dt
   (바이어스 행: F[3..5][3..5] = I, 나머지 0)

4. P 전파: P = F * P * F^T + Q
   Q = diag(q_angle, q_angle, q_angle, q_bias, q_bias, q_bias)

### Update A — accel (roll/pitch)
예상 가속도 (중력 기준, g=9.81):
  h[0] = -sinf(x[1]) * 9.81f
  h[1] =  cosf(x[1]) * sinf(x[0]) * 9.81f
  h[2] =  cosf(x[1]) * cosf(x[0]) * 9.81f

H_a (3×6, 0으로 초기화 후 아래만 채움):
  H_a[0][1] = -cosf(x[1]) * 9.81f
  H_a[1][0] =  cosf(x[1]) * cosf(x[0]) * 9.81f
  H_a[1][1] = -sinf(x[1]) * sinf(x[0]) * 9.81f
  H_a[2][0] = -cosf(x[1]) * sinf(x[0]) * 9.81f
  H_a[2][1] = -sinf(x[1]) * cosf(x[0]) * 9.81f

skip 조건: fabsf(norm3(ax,ay,az)*9.81f - 9.81f) > ACCEL_THRESH
           또는 fabsf(x[1]) > 85°*DEG2RAD

### Update B — mag (yaw)
R = dcmFromEuler(x[0], x[1], x[2])  → R_body2world의 전치로 레퍼런스 회전
  h_m = R_world2body * [mx_ref, my_ref, mz_ref]^T

H_m (3×6): ∂h_m/∂x의 해석적 계산
  (dcmFromEuler의 φ,θ,ψ 편미분 전개, 수동 계산)

skip 조건: fabsf(norm3(mx,my,mz) - norm3(mx_ref,my_ref,mz_ref)) > MAG_THRESH
           또는 fabsf(x[1]) > 85°*DEG2RAD

### 공통 Kalman Update 함수 (내부용)
void ekfKalmanUpdate(EkfState& s,
                     const float H[3][6],
                     const float z[3],
                     const float h[3],
                     float r_noise);
  S    = H*P*H^T + R  (3×3, R = r_noise * I)
  K    = P*H^T * S^-1 (6×3)
  δx   = K * (z - h)  (6×1)
  x   += δx
  P    = (I - K*H) * P

### 인터페이스
void ekfInit(EkfState& s, float phi0, float theta0, float psi0,
             float mx_ref, float my_ref, float mz_ref);
void ekfPredict(EkfState& s, float gx_rad, float gy_rad, float gz_rad, float dt);
void ekfUpdateAccel(EkfState& s, float ax, float ay, float az);
void ekfUpdateMag(EkfState& s, float mx, float my, float mz);
EulerAngle ekfGetEuler(const EkfState& s);

────────────────────────────────────────
## Serial CSV 출력 형식
────────────────────────────────────────
헤더 1행 후 데이터. 활성 필터만 컬럼 포함.
소수점 2자리, 각도 [deg], 원시센서 원래 단위.

time_ms,
ax,ay,az,
gx_dps,gy_dps,gz_dps,
mx,my,mz,
phi_gyro,theta_gyro,psi_gyro,
phi_ekf,theta_ekf,psi_ekf,
phi_cf,theta_cf,psi_cf,
phi_mahony,theta_mahony,psi_mahony,
phi_madgwick,theta_madgwick,psi_madgwick

────────────────────────────────────────
## main.cpp 루프 구조
────────────────────────────────────────
setup():
  1. imu.begin()
  2. 첫 샘플로 accelRollPitch() → phi0, theta0
  3. magYaw()로 psi0
  4. 각 필터 Init (phi0, theta0, psi0 전달)
  5. CSV 헤더 출력

loop():
  1. imu.update(), applyBias()
  2. gyroX/Y/Z를 DEG2RAD 변환
  3. 각 필터 Predict(gx_rad, gy_rad, gz_rad, Ts)
  4. 각 필터 UpdateAccel(ax, ay, az)
  5. 각 필터 UpdateMag(mx, my, mz)
  6. Serial: time, raw센서, 각 필터 Euler [deg] 순서로 출력
  7. delay(LOOP_MS)