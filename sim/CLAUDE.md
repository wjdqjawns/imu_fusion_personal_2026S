# AHRS Simulation — 개발 가이드 프롬프트

> **사용법**: 이 문서를 Claude에게 붙여넣고 원하는 작업을 이어서 요청하세요.  
> 코드 구조·파이프라인 흐름·설계 의도가 모두 담겨 있습니다.

---

## 프로젝트 개요

Python으로 구현된 멀티스테이지 AHRS(Attitude and Heading Reference System) 시뮬레이션 파이프라인입니다.  
IMU 센서 노이즈 특성 분석 → Ground Truth 궤적 생성 → 상태 추정 필터 비교 → 정량 평가 → 시각화 및 리포트 생성의 흐름으로 동작합니다.

---

## 코드 구조

```
src/ahrs/
├── core/
│   ├── calibration/        # mag_cal.py (hard/soft iron), accel_cal.py, gyro_cal.py
│   ├── env/                # environment.py, gravity_field.py, magnetic_field.py, thermal_field.py
│   ├── estimator/          # base.py, complementary.py, mahony.py, madgwick.py,
│   │                       # ekf_euler.py, ekf_quat.py, eskf.py
│   ├── evaluation/         # metrics.py, allan.py, psd.py, fft.py, time.py
│   ├── model/              # sensor.py (IMUSensor, GroundTruth), gyroscope.py, accelerometer.py, magnetometer.py
│   ├── noise/              # profile.py, white.py, colored.py
│   ├── orientation/        # quaternion.py, euler.py, dcm.py, transforms.py
│   └── trajectory/         # base.py, static.py, circular.py, figure8.py, spherical.py
├── pipeline/
│   └── runner.py           # 메인 파이프라인 오케스트레이터
└── utils/
    ├── configger.py        # YAML 로드
    ├── plotter.py          # 모든 시각화
    ├── reporter.py         # FilterReport (MD/JSON)
    └── logger.py
```

**출력 디렉터리 구조** (`export/{timestamp}/`):
```
data/   truth.csv, {filter}.csv, noise_static.csv, noise_sphere.csv
fig/    allan_*.png, psd_*.png, mag_calibration.png,
        attitude_comparison.png, attitude_error.png,
        geodesic_error.png, rmse_bar.png, trajectory_3d.png,
        convergence.png, freq_sweep_error.png, gt_vs_est_scatter.png
log/    sim.log
report/ report_{ts}.md, report_{ts}.json
```

---

## 파이프라인 7단계 흐름

### Stage 0 — Config 로드
- `load_config(config_path)` → `cfg: dict`
- 핵심 키: `simulation`, `trajectory`, `filters`, `imu`, `calibration`, `env`, `output`

### Stage 1 — Simulator 세팅
- `dt`, `duration`, `random_seed` 추출
- output dir 생성, 파일 핸들러 로거 attach, plotter/exporter 빌드

### Stage 2 — Environment 세팅
- `Environment.from_config(cfg["env"])`
- 중력 크기, 자기장 NED 벡터, 열적 모델 초기화
- `IMUSensor.from_config(cfg, seed=seed)` — 센서 노이즈 프로파일 설정

### Stage 3 — Sensor Characterization (`_run_noise_stage`)

#### 3-A. 정적 궤적 (Static path) — Allan Deviation + PSD + FFT
```
StaticTrajectory().generate(dt, duration_s)
    → _collect_raw() → gyro/accel/mag raw 수집
    → AllanVariance(data, dt).compute()   # tau, adev
    → .extract_params()                   # ARW, BI, RRW per axis
    → plotter.allan_variance(tau, adev)
    → plotter.psd(data, fs)               # PSD (Welch)
    → plotter.fft(data, fs)               # FFT 스펙트럼
    → data/noise_static.csv 저장
```

**Allan Deviation 해석 포인트**:
- 기울기 `-0.5` 구간 → ARW (Angle Random Walk)
- 기울기 `0` (최솟값) → Bias Instability
- 기울기 `+0.5` 구간 → RRW (Rate Random Walk)

**PSD / FFT 포인트**:
- 플랫 스펙트럼 → 백색 잡음 지배
- `1/f` 경사 → flicker noise (long-term drift)
- 특정 주파수 스파이크 → 진동 간섭 의심

#### 3-B. 구형 궤적 (Spherical path) — Hard/Soft Iron 보정
```
SphericalTrajectory(rate_deg_s=30).generate(dt, duration_s)
    → _collect_raw() → mag raw 수집
    → MagCalibrator(method="ellipsoid_fit").fit(mag_raw)
    → .apply(m)  # hard iron offset 제거, soft iron matrix 적용
    → plotter.mag_sphere(raw, calibrated)  # 3D 구 분포 비교
    → data/noise_sphere.csv 저장
```

**Hard iron**: 상수 오프셋 → 구의 중심이 원점에서 벗어남  
**Soft iron**: 축 스케일/왜곡 → 구가 타원체로 변형됨

### Stage 4 — Trajectory 생성 (Ground Truth)

#### 지원 궤적 타입
| 타입 | 설명 | 용도 |
|---|---|---|
| `static` | 정지 상태 | 노이즈 분석 기준선 |
| `circular` | 1축 원형 회전 | 단순 회전 테스트 |
| `figure8` | 2축 복합 궤적 | 일반 자세 추정 |
| `spherical` | 구면 전방향 회전 | 자력계 캘리브레이션 |
| `sinusoidal` | 축별 sin 함수 궤적 | **주파수 응답 분석** |

#### Sinusoidal 궤적 설계 (권장 추가 사항)
```python
# 단일 주파수
roll(t)  = A_r * sin(2π * f_r * t)
pitch(t) = A_p * sin(2π * f_p * t + φ_p)
yaw(t)   = A_y * sin(2π * f_y * t + φ_y)

# 주파수 스윕 (chirp)
roll(t)  = A * sin(2π * f(t) * t)
f(t)     = f_start + (f_end - f_start) * t / T
```

**주파수 분석 실험 예시**:
```yaml
# config.yaml
trajectory:
  type: sinusoidal
  sinusoidal:
    frequencies_hz: [0.1, 0.5, 1.0, 2.0, 5.0]  # sweep 대상
    amplitude_deg: 30.0
    axes: [roll, pitch, yaw]
    phase_offset_deg: [0, 45, 90]
```

**샘플링 시점 마킹**: 이산 측정 시점을 플롯에 점으로 표시
```python
# plotter.attitude_comparison()에서
ax.plot(t, truth_euler[:,0], 'b-', label='GT', linewidth=1.2)
ax.plot(t_sampled, noisy_euler[sample_idx, 0], 'b.', markersize=4,
        label='noisy GT (measured)', alpha=0.6)
ax.plot(t, est_euler[:,0], 'r-', label='EKF', linewidth=1.0)
```

### Stage 5 — 센서 측정 + 상태 추정 (`_run_estimation_stage`)

#### 추정 루프 구조
```python
for k, t_k in enumerate(truth.t):
    # Ground Truth → 센서 측정값 생성
    gt   = GroundTruth(q=truth.q[k], omega=truth.omega[k], ...)
    meas = sensor.measure(gt, t_k, dt)

    # disturbance 추가 (선형 가속도 / 자기 간섭)
    accel_meas = meas.accel + linear_disturbance(t_k)
    mag_meas   = meas.mag   + magnetic_disturbance(t_k)

    # 각 필터 업데이트
    for name, filt in filters.items():
        q, bias = filt.update(meas.gyro, accel_meas, mag_meas, dt)
        filt_results[name]["q"][k]    = q
        filt_results[name]["bias"][k] = bias
```

#### 지원 필터
| 필터 | 상태벡터 | 특이점 | Manifold | 특징 |
|---|---|---|---|---|
| `complementary` | — | 없음 | ✗ | 가장 빠르고 단순, 낮은 계산 비용 |
| `mahony` | — | 없음 | ✗ | PI 기반, 자기장 보정 포함 |
| `madgwick` | — | 없음 | ✗ | gradient descent, 임베디드 최적화 |
| `ekf_euler` | `[φ,θ,ψ, b_g]` 6D | **있음** (pitch ±90°) | ✗ | 교육용 기준선, 특이점 체험 |
| `ekf_quat` | `[q_w..z, b_g]` 7D | 없음 | 부분적 (renorm) | 확률 추정 기준선 |
| `eskf` | nominal `[q̂,b̂_g]` + error `δx` 6D | 없음 | ✓ (tangent space) | 실용 AHRS 표준 |

#### 필터 파일 구조
```
core/estimator/
├── base.py
├── complementary.py
├── mahony.py
├── madgwick.py
├── ekf_euler.py      # Euler-EKF (신규)
├── ekf_quat.py       # 기존 ekf.py → 이름 변경
└── eskf.py           # ESKF (신규)
```

#### Euler-EKF 수학 구조

상태벡터: `x = [φ, θ, ψ, b_gx, b_gy, b_gz]ᵀ` (6×1)

**Process model** — 오일러 각 시간 미분:
```
[φ̇]   [1  sin φ tan θ       cos φ tan θ    ] [p - b_gx]
[θ̇] = [0    cos φ            -sin φ         ] [q - b_gy]  = W(φ,θ) · ω̃
[ψ̇]   [0  sin φ / cos θ    cos φ / cos θ   ] [r - b_gz]
```

**특이점**: `cos θ → 0` (pitch ±90°) 시 `W` 발산 → F, P 발산 → 필터 붕괴  
**Jacobian** F (6×6): `∂(W·ω)/∂φ`, `∂(W·ω)/∂θ` 해석적 계산

#### EKF (Quaternion) 수학 구조

상태벡터: `x = [q_w, q_x, q_y, q_z, b_gx, b_gy, b_gz]ᵀ` (7×1)

**Process model**: `q̇ = ½ q ⊗ [0, ω̃]ᵀ`  
**제약**: update 후 `‖q‖ ≠ 1` → 매 스텝 renormalize 필요  
**문제**: 공분산 P가 SO(3) 구조를 무시 → 불확실성 표현 왜곡

#### ESKF 수학 구조

상태를 두 층으로 분리:
```
nominal state  x̂ = [q̂, b̂_g]   ← IMU 적분으로 전파
error state    δx = [δθ, δb_g]  ← Kalman이 추정 (6×1, 유클리드)
```

**Predict**:
```
q̂_k = q̂_{k-1} ⊗ Exp((ω̃ - b̂_g) · dt)
F    = jacobian of error dynamics (6×6)
P    = F P Fᵀ + Q
```

**Update** (accel / mag 관측):
```
H  = jacobian of h(x̂) w.r.t. δx   (관측 함수의 접선 공간 Jacobian)
K  = P Hᵀ (H P Hᵀ + R)⁻¹
δx = K (z - h(x̂))
```

**Inject & Reset** (manifold composition):
```
q̂    ← q̂ ⊗ Exp(δθ / 2)
b̂_g  ← b̂_g + δb_g
δx   ← 0
P    ← (I - KH) P
```

`Exp(δθ)`: 회전벡터 → 쿼터니언 (`transforms.py`의 `rotvec_to_quat` 활용)

#### 플롯 요구사항
```
attitude_comparison.png
  - x축: 시간(s)
  - y축: roll/pitch/yaw (deg), 3 subplot
  - 라인: GT(실선) / noisy GT(점) / 각 필터(색상별 실선)
  - disturbance 구간: 음영 강조

freq_sweep_error.png
  - x축: 주파수 (Hz, log scale)
  - y축: RMSE (deg)
  - 필터별 곡선 → 주파수 응답 특성 비교

gt_vs_est_scatter.png
  - x축: GT angle (deg)
  - y축: Estimated angle (deg)
  - 이상적 선: y = x (점선)
  - 필터별 산점도 → 편향/분산 시각화
```

### Stage 6 — Evaluation

#### 지표
```python
euler_rmse(est_euler, gt_euler)       # [roll, pitch, yaw] RMSE (deg)
geodesic_rmse(est_q, gt_q)            # 쿼터니언 기반 측지 오차 (deg)
geodesic_errors(est_q, gt_q)          # 시간별 오차 시계열
attitude_error_euler(est_q, gt_q)     # [err_roll, err_pitch, err_yaw] 시계열
```

#### 주파수 스윕 평가 흐름
```python
freq_rmse = {}
for freq in cfg["trajectory"]["sinusoidal"]["frequencies_hz"]:
    cfg_f = deepcopy(cfg)
    cfg_f["trajectory"]["sinusoidal"]["frequencies_hz"] = [freq]
    truth_f  = build_trajectory(cfg_f).generate(dt, duration)
    est_f    = run_filters(truth_f, sensor, filters, env, dt)
    freq_rmse[freq] = {name: geodesic_rmse(est_f[name]["q"], truth_f.q)
                       for name in filters}
```

### Stage 7 — Plot + Report

#### 필수 플롯 (Plotter 메서드)
```python
plotter.attitude_comparison(t, truth_euler, filter_euler,
                             sample_times=t_sampled,          # 이산 측정 시점 마킹
                             disturbance_spans=dist_spans)
plotter.attitude_error(t, filter_errors, disturbance_spans=dist_spans)
plotter.geodesic_error(t, geo_errors, disturbance_spans=dist_spans)
plotter.rmse_bar(rmse_table)
plotter.trajectory_3d(truth_euler, filter_euler)
plotter.convergence(t, geo_errors)
plotter.freq_sweep_error(freq_rmse)         # 주파수별 RMSE 곡선
plotter.gt_vs_est_scatter(truth_euler,      # y=x 산점도
                           filter_euler)
# Noise stage
plotter.allan_variance(tau, adev, axis_labels, out_path)
plotter.psd(data, fs, title, axis_labels, unit_label, out_path)
plotter.fft_spectrum(data, fs, out_path)
plotter.mag_sphere(raw, calibrated, out_path)
```

#### 리포트 구조 (FilterReport)
```
report_{ts}.md
  ├── 실행 환경 (dt, duration, seed, env 파라미터)
  ├── [Stage 3] Sensor Characterization
  │   ├── Allan Deviation 표 (ARW, BI, RRW per axis)
  │   ├── Mag Calibration 결과 (hard iron offset, soft iron matrix)
  │   └── 노이즈 프로파일 요약
  ├── [Stage 5-6] Estimation Results
  │   ├── 필터별 RMSE 표 (roll / pitch / yaw / geodesic)
  │   ├── 주파수별 오차 표 (freq sweep 실행 시)
  │   └── 수렴 시간 분석
  └── 결론 및 필터 추천
```

---

## Runner 리팩토링 가이드

현재 `runner.py`는 stage별 책임이 혼재되어 있습니다. 다음 방향으로 정리하세요:

### 권장 모듈 분리

```
pipeline/
├── runner.py           # run() 진입점만. 각 stage 함수 호출 오케스트레이터
├── stage_noise.py      # _run_noise_stage() + _collect_raw() + CSV helpers
├── stage_estimation.py # _run_estimation_stage() + main loop + disturbance logic
├── stage_evaluation.py # eval_results 계산, freq sweep loop
├── stage_report.py     # plot 호출 + FilterReport 저장
└── builders.py         # build_env / build_trajectory / build_estimator / build_plotter
```

### runner.py 골격 (리팩토링 후)

```python
def run(config_path, dry_run=False, log_level="INFO"):
    cfg     = load_config(config_path)
    out_dir = _make_output_dir(cfg)
    _setup_logging(out_dir, log_level)

    dt       = float(cfg["simulation"]["dt"])
    duration = float(cfg["simulation"]["duration"])
    seed     = int(cfg["simulation"].get("random_seed", 42))

    env      = build_env(cfg)
    sensor   = build_sensor(cfg, seed)
    plotter  = build_plotter(cfg)

    # Stage 3
    noise_results = {}
    if cfg.get("calibration", {}).get("enabled", True):
        noise_results = run_noise_stage(cfg, env, sensor, dt, out_dir, plotter)

    # Stage 4-6
    est = run_estimation_stage(cfg, env, sensor, dt, duration, out_dir, plotter)

    # Stage 7
    run_report_stage(est, noise_results, cfg, out_dir, plotter)

    return {"truth": est["truth"], "results": est["results"],
            "noise": noise_results, "config": cfg, "out_dir": out_dir}
```

---

## Config 예시 (핵심 항목)

```yaml
simulation:
  dt: 0.01
  duration: 60.0
  random_seed: 42

env:
  latitude_deg: 37.5
  altitude_m: 100.0
  magnetic_declination_deg: -8.0

trajectory:
  type: sinusoidal           # static | circular | figure8 | spherical | sinusoidal
  sinusoidal:
    frequencies_hz: [0.1, 0.5, 1.0, 2.0, 5.0]
    amplitude_deg: 30.0
    axes: [roll, pitch, yaw]
    phase_offset_deg: [0, 45, 90]

calibration:
  enabled: true
  allan_variance:
    enabled: true
    duration_s: 300.0
  magnetometer:
    enabled: true
    duration_s: 120.0
    method: ellipsoid_fit    # ellipsoid_fit | sphere_fit

filters:
  run: [complementary, mahony, madgwick, ekf_euler, ekf_quat, eskf]
  complementary:
    alpha: 0.98
  mahony:
    kp: 2.0
    ki: 0.005
  madgwick:
    beta: 0.1
  ekf_euler:
    sigma_gyro: 0.001
    sigma_accel: 0.01
    sigma_mag: 0.5
  ekf_quat:
    sigma_gyro: 0.001
    sigma_accel: 0.01
    sigma_mag: 0.5
  eskf:
    sigma_gyro: 0.001
    sigma_accel: 0.01
    sigma_mag: 0.5
    sigma_gyro_bias: 0.0001

imu:
  disturbance:
    linear_acceleration:
      enabled: true
      events:
        - onset_time_s: 20.0
          duration_s: 2.0
          direction_body: [1, 0, 0]
          magnitude_g: 0.3
    magnetic:
      enabled: true
      events:
        - onset_time_s: 40.0
          duration_s: 3.0
          disturbance_uT: [50, 0, 0]

output:
  base_dir: export
  timestamp_dirs: true
  fig:
    dpi: 150
    format: png
```

---

## 자주 쓰는 작업별 프롬프트 템플릿

### 1. 특정 stage 구현 요청
```
[AHRS SIM] Stage 3-A를 구현해줘.
AllanVariance.extract_params()가 ARW, BI, RRW를 각 축별로 반환하고,
plotter.allan_variance()에서 tau vs ADEV를 log-log 스케일로 그려줘.
기울기 라인(-0.5, 0, +0.5)도 표시해.
```

### 2. 새 궤적 타입 추가
```
[AHRS SIM] SinusoidalTrajectory 클래스를 추가해줘.
core/trajectory/sinusoidal.py에 만들고, TruthData를 반환해.
주파수 스윕(frequencies_hz 리스트)을 지원하고,
각 주파수 구간을 이어붙여 하나의 연속 궤적으로 만들어줘.
```

### 3. 플롯 추가
```
[AHRS SIM] plotter.py에 gt_vs_est_scatter() 메서드를 추가해줘.
x축: GT angle (deg), y축: Estimated angle (deg)
y=x 이상적 직선을 점선으로 그리고,
각 필터별 산점도를 subplot(1,3)으로 roll/pitch/yaw 분리해서 보여줘.
```

### 4. runner.py 리팩토링
```
[AHRS SIM] runner.py를 아래 구조로 분리해줘:
- pipeline/builders.py : build_env, build_trajectory, build_estimator, build_plotter
- pipeline/stage_noise.py : _run_noise_stage, _collect_raw, CSV helpers
- pipeline/stage_estimation.py : _run_estimation_stage, main loop
- pipeline/stage_evaluation.py : freq sweep loop, eval metrics
- pipeline/stage_report.py : 모든 plotter 호출 + FilterReport 저장
runner.py는 run() 진입점과 오케스트레이션만 남겨줘.
```

### 5. 주파수 응답 평가
```
[AHRS SIM] 주파수 스윕 평가 루프를 stage_evaluation.py에 추가해줘.
frequencies_hz 리스트의 각 주파수에 대해 독립적으로 시뮬레이션 돌리고,
필터별 geodesic RMSE를 freq_rmse[freq][filter_name] 딕셔너리에 저장해.
plotter.freq_sweep_error(freq_rmse)로 log-x 스케일 라인 플롯 그려줘.
```

### 6. Euler-EKF 구현
```
[AHRS SIM] core/estimator/ekf_euler.py를 구현해줘.
AhrsFilter 베이스 클래스를 상속하고, update(gyro, accel, mag, dt) 인터페이스를 맞춰.
상태벡터: x = [φ, θ, ψ, b_gx, b_gy, b_gz] (6×1, 단위 rad)
Process model: ẋ_att = W(φ,θ) · (ω_meas - b_g), Jacobian F 해석적으로 계산.
Observation: accel → gravity 방향으로 roll/pitch 보정, mag → yaw 보정.
pitch ±90° 근방(|cos θ| < 1e-4)에서 W 발산 방지 처리 포함.
반환값: (q_from_euler, bias) — euler를 쿼터니언으로 변환해서 반환.
```

### 7. ESKF 구현
```
[AHRS SIM] core/estimator/eskf.py를 구현해줘.
AhrsFilter 베이스 클래스를 상속하고, update(gyro, accel, mag, dt) 인터페이스를 맞춰.
nominal state: q̂ (scalar-first quaternion), b̂_g (gyro bias)
error state:   δx = [δθ (3), δb_g (3)] (6×1)
공분산:        P ∈ ℝ⁶ˣ⁶

Predict:
  q̂ ← q̂ ⊗ Exp((ω̃ - b̂_g) · dt)   # Exp = rotvec_to_quat (transforms.py)
  F  = error dynamics Jacobian (6×6)
  P  ← F P Fᵀ + Q

Update (accel + mag 순차 적용):
  H  = 관측 함수의 접선 공간 Jacobian
  K  = P Hᵀ (H P Hᵀ + R)⁻¹
  δx = K (z - h(x̂))

Inject & Reset:
  q̂   ← normalize(q̂ ⊗ Exp(δθ / 2))
  b̂_g ← b̂_g + δb_g
  P   ← (I - KH) P
  δx  ← 0

반환값: (q̂, b̂_g)
```

---

## 필터 비교 실험 시나리오

세 EKF 계열 필터의 차이가 잘 드러나는 조건:

| 시나리오 | 설정 | 기대 결과 |
|---|---|---|
| **정상 운동** (pitch < 45°) | figure8, amplitude 30° | 세 필터 비슷 → 복잡도 대비 성능 비교 기준선 |
| **고피치 운동** (pitch → ±80°) | sinusoidal, pitch amplitude 80° | Euler-EKF만 발산 → 특이점 시각화 |
| **고주파 운동** (> 2Hz) | sinusoidal freq sweep | Euler-EKF linearization 오차 누적 vs ESKF 견고함 |
| **자이로 바이어스 주입** | ekf.sigma_gyro_bias 크게 설정 | 바이어스 수렴 속도 비교 (bias subplot) |
| **자기 간섭 disturbance** | magnetic.events 활성화 | yaw 복원 시간 비교 |

**Euler-EKF 특이점 트리거 조건**:
```yaml
trajectory:
  type: sinusoidal
  sinusoidal:
    frequencies_hz: [0.3]
    amplitude_deg: 85.0      # pitch가 ±85°까지 → cos θ ≈ 0.09
    axes: [roll, pitch, yaw]
```

**비교 플롯 체크리스트**:
- `attitude_comparison.png` — Euler-EKF가 발산하는 구간 음영 표시
- `geodesic_error.png` — 세 EKF의 오차 분포 비교
- `rmse_bar.png` — 시나리오별 RMSE 테이블
- `freq_sweep_error.png` — 주파수별 각 필터 열화 특성

---

## 설계 원칙 및 주의사항

- **단방향 의존성**: `core/` → `utils/`만 허용. `utils/`가 `core/`를 import하면 안 됨
- **TruthData 불변성**: trajectory가 생성한 GT는 수정 금지. 사본 사용
- **시드 고정**: 재현성을 위해 `np.random.seed(seed)` 반드시 runner 진입 시 설정
- **단위 일관성**: 각도는 내부적으로 rad, 출력만 deg 변환. 혼용 금지
- **quaternion 컨벤션**: scalar-first `[w, x, y, z]` 통일
- **disturbance 처리**: 센서 측정값에 추가(post-measure), GT에는 반영하지 않음
- **플롯 저장**: `out_path`가 None이면 화면 표시, 있으면 파일 저장. 두 경우 모두 지원
- **CSV 헤더**: 단위를 헤더에 명시 (`roll_deg`, `gx_rad_s`, `mx_uT`)

---

*Author: Beomjun Chung | Updated: 2026-05-25 (Euler-EKF / EKF-Quat / ESKF 추가)*