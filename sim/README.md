# IMU Fusion Simulation

Python으로 구현된 멀티스테이지 AHRS(Attitude and Heading Reference System) 시뮬레이션 파이프라인.

IMU 센서 노이즈 분석 → Ground Truth 궤적 생성 → 상태 추정 필터 비교 → 정량 평가 → 시각화 및 리포트

---

## 목차

1. [설치](#설치)
2. [빠른 시작](#빠른-시작)
3. [실행 방법](#실행-방법)
4. [출력 구조](#출력-구조)
5. [Config 설정 가이드](#config-설정-가이드)
   - [궤적 타입 변경](#궤적-타입-변경)
   - [필터 선택](#필터-선택)
   - [IMU 노이즈 파라미터](#imu-노이즈-파라미터)
   - [외란 이벤트](#외란-이벤트)
   - [초기 조건](#초기-조건-수렴-속도-비교)
   - [노이즈 분석 설정](#노이즈-분석-allan-variance)
   - [환경 설정](#환경-설정)
6. [필터 비교 실험 시나리오](#필터-비교-실험-시나리오)
7. [프로젝트 구조](#프로젝트-구조)
8. [좌표계 및 컨벤션](#좌표계-및-컨벤션)

---

## 설치

Python 3.10 이상 필요.

```bash
# 패키지 설치 (editable mode — 코드 수정이 즉시 반영됨)
pip install -e .

# 선택사항: IEEE/Nature 스타일 플롯
pip install SciencePlots
```

---

## 빠른 시작

```bash
# 기본 설정으로 시뮬레이션 실행
py -m ahrs.main

# config 검증만 (시뮬 없이)
py -m ahrs.main --dry-run

# 커스텀 config 파일 지정
py -m ahrs.main --config config/my_experiment.yaml

# 로그 상세 출력
py -m ahrs.main --log-level DEBUG
```

실행 완료 후 결과는 `export/YYYY-MM-DD_HHMMSS/` 디렉토리에 자동 저장됩니다.

---

## 실행 방법

### 1. 메인 시뮬레이션

```bash
py -m ahrs.main [옵션]
```

| 옵션 | 기본값 | 설명 |
|---|---|---|
| `--config PATH` | `config/config.yaml` | 설정 파일 경로 |
| `--dry-run` | — | config 검증만, 시뮬 미실행 |
| `--log-level LEVEL` | `INFO` | `DEBUG` / `INFO` / `WARNING` |

### 2. 저장된 결과로 플롯만 재생성

시뮬을 다시 실행하지 않고 CSV 결과에서 그래프만 다시 그릴 때 사용합니다.

```bash
# 가장 최근 run 자동 선택
py scripts/run_compare.py

# 특정 run 디렉토리 지정
py scripts/run_compare.py --run-dir export/2026-05-24_143022

# 화면에도 팝업 표시
py scripts/run_compare.py --show

# 고해상도 Nature 스타일
py scripts/run_compare.py --style nature --dpi 300
```

### 3. Python 코드에서 직접 호출

```python
from ahrs.pipeline.runner import run

result = run("config/config.yaml", log_level="WARNING")

# 반환값
truth   = result["truth"]      # TruthData: GT 시계열 (q, omega, ...)
results = result["results"]    # {filter_name: {q, bias, rmse_deg, geodesic_rmse_deg, ...}}
noise   = result["noise"]      # Allan variance, mag calibration 결과
out_dir = result["out_dir"]    # 저장 경로 (Path 객체)
```

---

## 출력 구조

```
export/
└── 2026-05-24_143022/
    ├── data/
    │   ├── truth.csv               ← GT + raw IMU 측정값 시계열
    │   ├── complementary.csv       ┐
    │   ├── madgwick.csv            │ 필터별 추정 결과
    │   ├── ekf_euler.csv           │ (roll/pitch/yaw + 오차 + bias)
    │   ├── eskf.csv                ┘
    │   ├── noise_static.csv        ← 정적 궤적 원시 데이터 (Allan 분석용)
    │   └── noise_sphere.csv        ← 구형 궤적 자기장 데이터 (캘리브레이션용)
    ├── fig/
    │   ├── attitude_comparison.png ← GT vs 필터별 roll/pitch/yaw 비교
    │   ├── attitude_error.png      ← 오차 시계열 (외란 구간 빨간 음영)
    │   ├── geodesic_error.png      ← 쿼터니언 기반 구면 거리 오차
    │   ├── rmse_bar.png            ← 필터별 RMSE 막대 차트
    │   ├── trajectory_3d.png       ← 3D 자세 궤적 (+ 단위 구면 체축 방향)
    │   ├── convergence.png         ← 초기 수렴 구간 확대 플롯
    │   ├── allan_gyro.png          ← 자이로 Allan Deviation (log-log)
    │   ├── allan_accel.png         ← 가속도계 Allan Deviation
    │   ├── psd_gyro.png            ← 자이로 PSD (Welch)
    │   ├── psd_accel.png
    │   └── mag_calibration.png     ← Hard/Soft Iron 보정 전후 구면 분포
    ├── log/
    │   └── sim.log
    └── report/
        ├── report_20260524_143022.md    ← 마크다운 요약 리포트
        └── report_20260524_143022.json
```

**CSV 헤더 단위 규칙:**
- `_deg` → [°], `_rad_s` → [rad/s], `_m_s2` → [m/s²], `_uT` → [μT]

---

## Config 설정 가이드

모든 설정은 `config/config.yaml` 하나로 제어합니다.  
시나리오별로 파일을 복사해 관리하는 것을 권장합니다.

```bash
cp config/config.yaml config/scenario_highpitch.yaml
py -m ahrs.main --config config/scenario_highpitch.yaml
```

---

### 궤적 타입 변경

`trajectory.type` 하나만 바꾸면 됩니다.

```yaml
trajectory:
  type: figure8    # ← static | circular | figure8 | spherical | sinusoidal
```

---

#### `static` — 정지 상태

```yaml
trajectory:
  type: static
  static:
    duration: 300.0    # Allan variance 분석은 300s 이상 권장
```

노이즈 특성 분석의 기준선. `calibration.enabled: true`와 함께 사용합니다.

---

#### `circular` — 일정 Yaw Rate 선회

```yaml
trajectory:
  type: circular
  circular:
    duration: 60.0
    angular_velocity_deg_s: 20.0   # yaw rate [deg/s]
    roll_deg: 10.0                 # 경사 선회 (0 = 수평 선회)
    pitch_deg: 0.0
```

Yaw 추정 성능 비교에 적합합니다.

---

#### `figure8` — 8자 복합 궤적 (기본값)

```yaml
trajectory:
  type: figure8
  figure8:
    duration: 60.0
    max_tilt_deg: 45.0   # 최대 roll/pitch 기울기 [deg]
    period_s: 8.0        # 8자 한 주기 [s]
```

일반적인 자세 추정 성능 비교의 표준 궤적입니다.

---

#### `spherical` — 구면 전방향 회전

```yaml
trajectory:
  type: spherical
  spherical:
    duration: 120.0
    rate_deg_s: 30.0
```

자력계 캘리브레이션(Hard/Soft Iron) 전용. 모든 방향을 골고루 커버합니다.

---

#### `sinusoidal` — Sin 함수 궤적 (주파수 스윕)

```yaml
trajectory:
  type: sinusoidal
  sinusoidal:
    frequencies_hz: [0.1, 0.5, 1.0, 2.0, 5.0]   # 스윕할 주파수 목록 [Hz]
    amplitude_deg: 30.0                            # 진폭 [deg]
    axes: [roll, pitch, yaw]                       # 활성화할 축
    phase_offset_deg: [0, 45, 90]                  # 축별 위상 오프셋 [deg]
    duration_per_freq_s: null                      # null → duration / 주파수 개수
```

필터의 **주파수 응답** 분석에 사용합니다. 주파수가 높아질수록 어떤 필터가 먼저 열화하는지 확인할 수 있습니다.

단일 주파수만 쓰려면:

```yaml
    frequencies_hz: [1.0]
    duration_per_freq_s: 60.0
```

**Euler-EKF 특이점 유발:**

```yaml
trajectory:
  type: sinusoidal
  sinusoidal:
    frequencies_hz: [0.3]
    amplitude_deg: 85.0      # pitch가 ±85°까지 → cos θ ≈ 0 → 발산
    axes: [roll, pitch, yaw]
```

---

### 필터 선택

`estimator.run` 리스트에 원하는 필터 이름만 넣으면 됩니다.

```yaml
estimator:
  run: [complementary, mahony, madgwick, ekf, ekf_euler, eskf]
```

| 이름 | 방식 | 상태 차원 | 특징 |
|---|---|---|---|
| `complementary` | 상보 필터 | — | 가장 단순, 계산량 최소 |
| `mahony` | PI 기반 | — | 자기장 보정, 임베디드 최적 |
| `madgwick` | Gradient descent | — | 단일 파라미터 β |
| `ekf` | EKF (쿼터니언) | 7D | 확률 추정 기준선, 매 스텝 renorm |
| `ekf_euler` | EKF (오일러 각) | 6D | 교육용, pitch ±90° 특이점 발생 |
| `eskf` | Error-State KF | 6D error | 실용 AHRS 표준, SO(3) manifold |

**파라미터 튜닝 가이드:**

```yaml
estimator:
  complementary:
    alpha: 0.98              # 클수록 자이로 의존 ↑, 저주파 보정 느림
    use_magnetometer: true

  mahony:
    kp: 1.8                  # 클수록 빠른 수렴, 노이즈 민감도 ↑
    ki: 0.08                 # bias 추정 속도 (0으로 두면 bias 미추정)

  madgwick:
    beta: 0.08               # 클수록 accel/mag 관측 가중치 ↑

  ekf:
    gyro_noise_std_rad_s: 0.003      # 작을수록 자이로 신뢰 ↑
    accel_meas_std_m_s2: 0.05        # 클수록 accel 관측 무시
    mag_meas_std_uT: 0.1
    accel_threshold_g: 0.2           # 이 이상 동적 가속도 → 업데이트 스킵

  ekf_euler:
    sigma_gyro: 0.003
    sigma_accel: 0.05
    sigma_mag: 0.5
    sigma_gyro_bias: 0.0005

  eskf:
    sigma_gyro: 0.003
    sigma_accel: 0.05
    sigma_mag: 0.5
    sigma_gyro_bias: 0.0001          # 작을수록 bias 추정 안정적, 느림
```

---

### IMU 노이즈 파라미터

```yaml
imu:
  gyroscope:
    bias_deg_s: [0.5, -0.3, 0.2]         # 상수 바이어스 [deg/s]
    arw_deg_per_sqrth: 0.15              # Angle Random Walk (Allan -1/2 기울기)
    bias_instability_deg_h: 3.0          # Bias Instability (Allan 최솟값)
    rrw_deg_per_h_sqrth: 0.05            # Rate Random Walk (Allan +1/2 기울기)
    bias_correlation_time_s: 100.0       # Gauss-Markov 상관 시간

  accelerometer:
    bias_g: [0.02, -0.01, 0.03]          # 상수 바이어스 [g]
    vrw_m_per_s_per_sqrth: 0.05          # Velocity Random Walk
    bias_instability_ug: 50.0            # [μg]

  magnetometer:
    hard_iron_uT: [5.0, -3.0, 2.0]       # 캘리브레이션이 제거해야 할 숨겨진 오프셋
    soft_iron_matrix:                     # 축 스케일/왜곡 행렬
      - [1.08,  0.02,  0.00]
      - [0.01,  0.95,  0.01]
      - [0.00, -0.02,  1.03]
    noise_std_uT: 0.1
```

노이즈를 줄여 이상적인 환경에서 필터만 비교하려면:

```yaml
  gyroscope:
    bias_deg_s: [0.0, 0.0, 0.0]
    arw_deg_per_sqrth: 0.01
    bias_instability_deg_h: 0.1
```

---

### 외란 이벤트

특정 시간 구간에 선형 가속도 또는 자기 간섭을 주입합니다.  
플롯에서 해당 구간이 **빨간 음영**으로 표시됩니다.

```yaml
imu:
  disturbance:
    linear_acceleration:
      enabled: true
      events:
        - onset_time_s: 12.0         # 외란 시작 [s]
          duration_s: 2.0            # 지속 시간 [s]
          magnitude_g: 0.3           # 크기 [g]
          direction_body: [1, 0, 0]  # body frame 방향 (전방)

        - onset_time_s: 35.0
          duration_s: 3.0
          magnitude_g: 0.5
          direction_body: [0, 1, 0]  # 우측

    magnetic:
      enabled: true
      events:
        - onset_time_s: 20.0
          duration_s: 5.0
          disturbance_uT: [8.0, -3.0, 2.0]   # body frame 기준 [μT]
```

외란 없이 실행하려면:

```yaml
    linear_acceleration:
      enabled: false
    magnetic:
      enabled: false
```

---

### 초기 조건 (수렴 속도 비교)

필터의 초기 자세 오차와 자이로 바이어스 오차를 설정합니다.  
크게 줄수록 필터가 수렴하는 과정을 `convergence.png`에서 명확하게 비교할 수 있습니다.

```yaml
initial_conditions:
  attitude_error_rad:
    roll:  0.1745    # ≈ 10°
    pitch: 0.0873    # ≈  5°
    yaw:   0.3491    # ≈ 20°

  gyro_bias_error_deg_s: [0.1, -0.1, 0.05]
```

초기 오차 없이 시작하려면:

```yaml
initial_conditions:
  attitude_error_rad:
    roll: 0.0
    pitch: 0.0
    yaw: 0.0
  gyro_bias_error_deg_s: [0.0, 0.0, 0.0]
```

---

### 노이즈 분석 (Allan Variance)

```yaml
calibration:
  enabled: true         # false 시 Stage 3 전체 스킵 → 실행 속도 대폭 단축

  allan_variance:
    enabled: true
    duration_s: 300.0   # 짧으면 긴 tau Allan curve 불안정. 최소 100s 권장
    n_tau: 100           # 로그 스케일 tau 포인트 수

  magnetometer:
    enabled: true
    duration_s: 120.0
    method: ellipsoid_fit   # ellipsoid_fit | sphere_fit
```

빠른 개발 반복 중에는 꺼두는 것이 좋습니다:

```yaml
calibration:
  enabled: false
```

---

### 환경 설정

```yaml
env:
  gravity:
    mode: wgs84          # wgs84 (위도·고도 보정) | manual
    lat_deg: 35.87       # 위도 (wgs84 모드)
    alt_m: 106.0

    # 수동 설정 예시:
    # mode: manual
    # ned_m_s2: [0, 0, 9.797]

  magnetic:
    mode: manual
    ned_uT: [29.8, -3.8, 42.5]   # [North, East, Down] [μT]
    # 서울: [29.5, -3.5, 44.0]
    # 부산: [29.2, -3.2, 41.8]
```

---

### 시뮬레이션 기본 설정

```yaml
simulation:
  dt: 0.01          # 샘플 주기 [s] (0.01 = 100 Hz)
  duration: 60.0    # 총 시뮬레이션 시간 [s]
  random_seed: 42   # 재현성 보장. 다른 값으로 바꾸면 다른 노이즈 실현값
```

---

## 필터 비교 실험 시나리오

### 시나리오 A — 정상 운동 기준선

```yaml
trajectory:
  type: figure8
  figure8:
    max_tilt_deg: 30.0
estimator:
  run: [complementary, mahony, madgwick, ekf, ekf_euler, eskf]
```

세 EKF 계열이 비슷한 성능 → 복잡도 대비 성능 차이 파악.

---

### 시나리오 B — Euler-EKF 특이점 시각화

```yaml
trajectory:
  type: sinusoidal
  sinusoidal:
    frequencies_hz: [0.3]
    amplitude_deg: 85.0    # pitch → ±85°, cos θ ≈ 0
    axes: [roll, pitch, yaw]
estimator:
  run: [ekf, ekf_euler, eskf]
```

`ekf_euler`만 발산, `ekf`와 `eskf`는 안정 → Gimbal Lock 현상 시각화.

---

### 시나리오 C — 주파수 응답 비교

```yaml
trajectory:
  type: sinusoidal
  sinusoidal:
    frequencies_hz: [0.1, 0.5, 1.0, 2.0, 5.0]
    amplitude_deg: 30.0
simulation:
  duration: 50.0     # 주파수당 10s
estimator:
  run: [complementary, madgwick, ekf_euler, eskf]
```

주파수가 높아질수록 `ekf_euler`의 RMSE가 `eskf`보다 빠르게 증가합니다.

---

### 시나리오 D — 자기 간섭 복원 비교

```yaml
trajectory:
  type: figure8
imu:
  disturbance:
    linear_acceleration:
      enabled: false
    magnetic:
      enabled: true
      events:
        - onset_time_s: 15.0
          duration_s: 10.0
          disturbance_uT: [30.0, 0.0, 0.0]   # 강한 자기 간섭
estimator:
  run: [mahony, madgwick, ekf, eskf]
```

Yaw 복원 시간 비교. `geodesic_error.png`의 빨간 음영 구간 이후 추이를 확인합니다.

---

### 시나리오 E — 수렴 속도 비교

```yaml
initial_conditions:
  attitude_error_rad:
    roll:  0.5236    # 30°
    pitch: 0.2618    # 15°
    yaw:   0.8727    # 50°
  gyro_bias_error_deg_s: [0.5, -0.5, 0.2]
simulation:
  duration: 30.0
calibration:
  enabled: false
estimator:
  run: [complementary, mahony, madgwick, ekf, eskf]
```

`convergence.png`에서 2° threshold 이하 수렴 시간을 직접 비교합니다.

---

## 프로젝트 구조

```
sim/
├── config/
│   └── config.yaml             ← 모든 설정의 단일 진입점
├── src/ahrs/
│   ├── core/
│   │   ├── calibration/        ← mag_cal, accel_cal, gyro_cal
│   │   ├── env/                ← 중력, 자기장, 열 환경 모델
│   │   ├── estimator/          ← 6개 AHRS 필터
│   │   │   ├── complementary.py
│   │   │   ├── mahony.py
│   │   │   ├── madgwick.py
│   │   │   ├── ekf.py          ← EKF (쿼터니언 7D)
│   │   │   ├── ekf_euler.py    ← EKF (오일러 각 6D, 특이점 포함)
│   │   │   └── eskf.py         ← Error-State KF
│   │   ├── evaluation/         ← metrics, allan, psd, fft, time
│   │   ├── model/              ← IMUSensor, Gyroscope, Accelerometer, Magnetometer
│   │   ├── noise/              ← 노이즈 프로파일 (white, colored)
│   │   ├── orientation/        ← quaternion, euler, dcm, transforms
│   │   └── trajectory/         ← static, circular, figure8, spherical, sinusoidal
│   ├── pipeline/
│   │   └── runner.py           ← 전체 파이프라인 오케스트레이터
│   └── utils/
│       ├── configger.py        ← YAML 로드 및 검증
│       ├── plotter.py          ← 모든 시각화 (IEEE/Nature 스타일)
│       ├── reporter.py         ← MD/JSON 리포트 생성
│       └── logger.py
├── scripts/
│   ├── run_compare.py          ← 저장된 CSV → 플롯 재생성
│   ├── analysis_noise.py       ← Allan variance 단독 분석
│   └── plot_results.py
├── export/                     ← 시뮬레이션 결과 (자동 생성)
├── CLAUDE.md                   ← 개발 가이드 (AI 협업용)
├── requirements.txt
└── pyproject.toml

├───archive
│   │   config.yaml
│   │   
│   ├───2026-05-18_155300
│   │   ├───data
│   │   │       complementary.csv
│   │   │       ekf.csv
│   │   │       madgwick.csv
│   │   │       mahony.csv
│   │   │       truth.csv
│   │   │       
│   │   ├───fig
│   │   │       attitude_comparison.png
│   │   │       attitude_error.png
│   │   │       convergence.png
│   │   │       geodesic_error.png
│   │   │       rmse_bar.png
│   │   │       trajectory_3d.png
│   │   │       
│   │   ├───log
│   │   │       sim.log
│   │   │       
│   │   └───report
│   │           report_20260518_155300.json
│   │           report_20260518_155300.md
│   │           
│   ├───2026-05-18_155315
│   │   ├───fig
│   │   │       allan_accel.png
│   │   │       allan_gyro.png
│   │   │       mag_calibration.png
│   │   │       psd_accel.png
│   │   │       psd_gyro.png
│   │   │       psd_mag.png
│   │   │       
│   │   └───log
│   │           noise.log
│   │           
│   ├───2026-05-18_160554
│   │   ├───data
│   │   │       complementary.csv
│   │   │       ekf.csv
│   │   │       madgwick.csv
│   │   │       mahony.csv
│   │   │       noise_sphere.csv
│   │   │       noise_static.csv
│   │   │       truth.csv
│   │   │       
│   │   ├───fig
│   │   │       allan_accel.png
│   │   │       allan_gyro.png
│   │   │       attitude_comparison.png
│   │   │       attitude_error.png
│   │   │       convergence.png
│   │   │       geodesic_error.png
│   │   │       mag_calibration.png
│   │   │       psd_accel.png
│   │   │       psd_gyro.png
│   │   │       psd_mag.png
│   │   │       rmse_bar.png
│   │   │       trajectory_3d.png
│   │   │       
│   │   ├───log
│   │   │       sim.log
│   │   │       
│   │   └───report
│   │           report_20260518_160607.json
│   │           report_20260518_160607.md
│   │           
│   ├───2026-05-18_160702
│   │   ├───data
│   │   │       complementary.csv
│   │   │       ekf.csv
│   │   │       madgwick.csv
│   │   │       mahony.csv
│   │   │       noise_sphere.csv
│   │   │       noise_static.csv
│   │   │       truth.csv
│   │   │       
│   │   ├───fig
│   │   │       allan_accel.png
│   │   │       allan_gyro.png
│   │   │       attitude_comparison.png
│   │   │       attitude_error.png
│   │   │       convergence.png
│   │   │       geodesic_error.png
│   │   │       mag_calibration.png
│   │   │       psd_accel.png
│   │   │       psd_gyro.png
│   │   │       psd_mag.png
│   │   │       rmse_bar.png
│   │   │       trajectory_3d.png
│   │   │       
│   │   ├───log
│   │   │       sim.log
│   │   │       
│   │   └───report
│   │           report_20260518_160713.json
│   │           report_20260518_160713.md
│   │           
│   ├───2026-05-18_161429
│   │   ├───data
│   │   │       complementary.csv
│   │   │       ekf.csv
│   │   │       madgwick.csv
│   │   │       mahony.csv
│   │   │       noise_sphere.csv
│   │   │       noise_static.csv
│   │   │       truth.csv
│   │   │       
│   │   ├───fig
│   │   │       allan_accel.png
│   │   │       allan_gyro.png
│   │   │       attitude_comparison.png
│   │   │       attitude_error.png
│   │   │       convergence.png
│   │   │       geodesic_error.png
│   │   │       mag_calibration.png
│   │   │       psd_accel.png
│   │   │       psd_gyro.png
│   │   │       psd_mag.png
│   │   │       rmse_bar.png
│   │   │       trajectory_3d.png
│   │   │       
│   │   ├───log
│   │   │       sim.log
│   │   │       
│   │   └───report
│   │           report_20260518_161441.json
│   │           report_20260518_161441.md
│   │           
│   ├───2026-05-18_165733
│   │   ├───data
│   │   │       noise_static.csv
│   │   │       
│   │   ├───fig
│   │   │       allan_accel.png
│   │   │       allan_gyro.png
│   │   │       
│   │   └───log
│   │           sim.log
│   │           
│   ├───2026-05-18_170551
│   │   ├───data
│   │   │       complementary.csv
│   │   │       ekf.csv
│   │   │       madgwick.csv
│   │   │       mahony.csv
│   │   │       noise_sphere.csv
│   │   │       noise_static.csv
│   │   │       truth.csv
│   │   │       
│   │   ├───fig
│   │   │       allan_accel.png
│   │   │       allan_gyro.png
│   │   │       attitude_comparison.png
│   │   │       attitude_error.png
│   │   │       convergence.png
│   │   │       geodesic_error.png
│   │   │       mag_calibration.png
│   │   │       psd_accel.png
│   │   │       psd_gyro.png
│   │   │       psd_mag.png
│   │   │       rmse_bar.png
│   │   │       trajectory_3d.png
│   │   │       
│   │   ├───log
│   │   │       sim.log
│   │   │       
│   │   └───report
│   │           report_20260518_170608.json
│   │           report_20260518_170608.md
│   │           
│   ├───2026-05-19_054352
│   │   ├───data
│   │   │       complementary.csv
│   │   │       ekf.csv
│   │   │       madgwick.csv
│   │   │       mahony.csv
│   │   │       noise_sphere.csv
│   │   │       noise_static.csv
│   │   │       truth.csv
│   │   │       
│   │   ├───fig
│   │   │       allan_accel.png
│   │   │       allan_gyro.png
│   │   │       attitude_comparison.png
│   │   │       attitude_error.png
│   │   │       convergence.png
│   │   │       geodesic_error.png
│   │   │       mag_calibration.png
│   │   │       psd_accel.png
│   │   │       psd_gyro.png
│   │   │       psd_mag.png
│   │   │       rmse_bar.png
│   │   │       trajectory_3d.png
│   │   │       
│   │   ├───log
│   │   │       sim.log
│   │   │       
│   │   └───report
│   │           report_20260519_054408.json
│   │           report_20260519_054408.md
│   │           
│   ├───2026-05-19_054421
│   │   ├───data
│   │   │       complementary.csv
│   │   │       ekf.csv
│   │   │       madgwick.csv
│   │   │       mahony.csv
│   │   │       noise_sphere.csv
│   │   │       noise_static.csv
│   │   │       truth.csv
│   │   │       
│   │   ├───fig
│   │   │       allan_accel.png
│   │   │       allan_gyro.png
│   │   │       attitude_comparison.png
│   │   │       attitude_error.png
│   │   │       convergence.png
│   │   │       geodesic_error.png
│   │   │       mag_calibration.png
│   │   │       psd_accel.png
│   │   │       psd_gyro.png
│   │   │       psd_mag.png
│   │   │       rmse_bar.png
│   │   │       trajectory_3d.png
│   │   │       
│   │   ├───log
│   │   │       sim.log
│   │   │       
│   │   └───report
│   │           report_20260519_054433.json
│   │           report_20260519_054433.md
│   │           
│   ├───2026-05-25_020205
│   │   ├───data
│   │   │       noise_sphere.csv
│   │   │       noise_static.csv
│   │   │       
│   │   ├───fig
│   │   │       allan_accel.png
│   │   │       allan_gyro.png
│   │   │       mag_calibration.png
│   │   │       psd_accel.png
│   │   │       psd_gyro.png
│   │   │       psd_mag.png
│   │   │       
│   │   └───log
│   │           sim.log
│   │           
│   ├───2026-05-25_020530
│   │   ├───data
│   │   │       complementary.csv
│   │   │       ekf.csv
│   │   │       ekf_euler.csv
│   │   │       eskf.csv
│   │   │       madgwick.csv
│   │   │       mahony.csv
│   │   │       noise_sphere.csv
│   │   │       noise_static.csv
│   │   │       truth.csv
│   │   │       
│   │   ├───fig
│   │   │       allan_accel.png
│   │   │       allan_gyro.png
│   │   │       attitude_comparison.png
│   │   │       attitude_error.png
│   │   │       convergence.png
│   │   │       geodesic_error.png
│   │   │       mag_calibration.png
│   │   │       psd_accel.png
│   │   │       psd_gyro.png
│   │   │       psd_mag.png
│   │   │       rmse_bar.png
│   │   │       trajectory_3d.png
│   │   │       
│   │   ├───log
│   │   │       sim.log
│   │   │       
│   │   └───report
│   │           report_20260525_020617.json
│   │           report_20260525_020617.md
│   │           
│   ├───2026-05-25_020710
│   │   ├───data
│   │   │       complementary.csv
│   │   │       ekf.csv
│   │   │       ekf_euler.csv
│   │   │       eskf.csv
│   │   │       madgwick.csv
│   │   │       mahony.csv
│   │   │       noise_sphere.csv
│   │   │       noise_static.csv
│   │   │       truth.csv
│   │   │       
│   │   ├───fig
│   │   │       allan_accel.png
│   │   │       allan_gyro.png
│   │   │       attitude_comparison.png
│   │   │       attitude_error.png
│   │   │       convergence.png
│   │   │       geodesic_error.png
│   │   │       mag_calibration.png
│   │   │       psd_accel.png
│   │   │       psd_gyro.png
│   │   │       psd_mag.png
│   │   │       rmse_bar.png
│   │   │       trajectory_3d.png
│   │   │       
│   │   ├───log
│   │   │       sim.log
│   │   │       
│   │   └───report
│   │           report_20260525_020727.json
│   │           report_20260525_020727.md
│   │           
│   ├───2026-05-25_020740
│   │   ├───data
│   │   │       complementary.csv
│   │   │       ekf.csv
│   │   │       ekf_euler.csv
│   │   │       eskf.csv
│   │   │       madgwick.csv
│   │   │       mahony.csv
│   │   │       noise_sphere.csv
│   │   │       noise_static.csv
│   │   │       truth.csv
│   │   │       
│   │   ├───fig
│   │   │       allan_accel.png
│   │   │       allan_gyro.png
│   │   │       attitude_comparison.png
│   │   │       attitude_error.png
│   │   │       convergence.png
│   │   │       geodesic_error.png
│   │   │       mag_calibration.png
│   │   │       psd_accel.png
│   │   │       psd_gyro.png
│   │   │       psd_mag.png
│   │   │       rmse_bar.png
│   │   │       trajectory_3d.png
│   │   │       
│   │   ├───log
│   │   │       sim.log
│   │   │       
│   │   └───report
│   │           report_20260525_020755.json
│   │           report_20260525_020755.md
│   │           
│   ├───2026-05-25_021100
│   │   ├───data
│   │   │       complementary.csv
│   │   │       ekf.csv
│   │   │       ekf_euler.csv
│   │   │       eskf.csv
│   │   │       madgwick.csv
│   │   │       mahony.csv
│   │   │       noise_sphere.csv
│   │   │       noise_static.csv
│   │   │       truth.csv
│   │   │       
│   │   ├───fig
│   │   │       allan_accel.png
│   │   │       allan_gyro.png
│   │   │       attitude_comparison.png
│   │   │       attitude_error.png
│   │   │       convergence.png
│   │   │       geodesic_error.png
│   │   │       mag_calibration.png
│   │   │       psd_accel.png
│   │   │       psd_gyro.png
│   │   │       psd_mag.png
│   │   │       rmse_bar.png
│   │   │       trajectory_3d.png
│   │   │       
│   │   ├───log
│   │   │       sim.log
│   │   │       
│   │   └───report
│   │           report_20260525_021117.json
│   │           report_20260525_021117.md
│   │           
│   ├───2026-05-25_023936
│   │   └───log
│   │           sim.log
│   │           
│   ├───2026-05-25_024157
│   │   └───log
│   │           sim.log
│   │           
│   ├───2026-05-25_024335
│   │   └───log
│   │           sim.log
│   │           
│   ├───2026-05-25_024441
│   │   └───log
│   │           sim.log
│   │           
│   ├───2026-05-25_024503
│   │   ├───data
│   │   │       complementary.csv
│   │   │       ekf.csv
│   │   │       ekf_euler.csv
│   │   │       eskf.csv
│   │   │       madgwick.csv
│   │   │       mahony.csv
│   │   │       noise_sphere.csv
│   │   │       noise_static.csv
│   │   │       truth.csv
│   │   │       
│   │   ├───fig
│   │   │       allan_accel.png
│   │   │       allan_gyro.png
│   │   │       attitude_comparison.png
│   │   │       attitude_error.png
│   │   │       convergence.png
│   │   │       fft_accel.png
│   │   │       fft_gyro.png
│   │   │       geodesic_error.png
│   │   │       mag_calibration.png
│   │   │       psd_accel.png
│   │   │       psd_gyro.png
│   │   │       psd_mag.png
│   │   │       rmse_bar.png
│   │   │       trajectory_3d.png
│   │   │       
│   │   ├───log
│   │   │       sim.log
│   │   │       
│   │   └───report
│   │           report_20260525_024521.json
│   │           report_20260525_024521.md
│   │           
│   ├───data
│   │       ekf.csv
│   │       madgwick.csv
│   │       mahony.csv
│   │       truth.csv
│   │       
│   ├───fig
│   │   │   attitude_compare.png
│   │   │   attitude_error_compare.png
│   │   │   mag_xy_raw.png
│   │   │   
│   │   ├───2026-05-18_154220
│   │   │       attitude_comparison.png
│   │   │       attitude_error.png
│   │   │       convergence.png
│   │   │       geodesic_error.png
│   │   │       rmse_bar.png
│   │   │       trajectory_3d.png
│   │   │       
│   │   └───2026-05-18_154234
│   │           attitude_comparison.png
│   │           attitude_error.png
│   │           convergence.png
│   │           geodesic_error.png
│   │           rmse_bar.png
│   │           trajectory_3d.png
│   │           
│   ├───orientation
│   │   │   dcm.py
│   │   │   euler.py
│   │   │   quaternion.py
│   │   │   __init__.py
│   │   │   
│   │   └───__pycache__
│   │           dcm.cpython-313.pyc
│   │           euler.cpython-313.pyc
│   │           quaternion.cpython-313.pyc
│   │           transforms.cpython-313.pyc
│   │           __init__.cpython-313.pyc
│   │           
│   └───runs
│       ├───20260525_043343
│       │   ├───data
│       │   ├───fig
│       │   ├───log
│       │   │       sim.log
│       │   │       
│       │   └───report
│       ├───20260525_043544
│       │   ├───data
│       │   ├───fig
│       │   ├───log
│       │   │       sim.log
│       │   │       
│       │   └───report
│       ├───20260525_043749
│       │   ├───data
│       │   ├───fig
│       │   ├───log
│       │   │       sim.log
│       │   │       
│       │   └───report
│       ├───20260525_044258
│       │   ├───data
│       │   ├───fig
│       │   ├───log
│       │   │       sim.log
│       │   │       
│       │   └───report
│       ├───20260525_044415
│       │   ├───data
│       │   ├───fig
│       │   ├───log
│       │   │       sim.log
│       │   │       
│       │   └───report
│       ├───20260525_044432
│       │   ├───data
│       │   ├───fig
│       │   ├───log
│       │   │       sim.log
│       │   │       
│       │   └───report
│       ├───20260525_050620
│       │   ├───data
│       │   ├───fig
│       │   ├───log
│       │   │       sim.log
│       │   │       
│       │   └───report
│       ├───20260525_050656
│       │   ├───data
│       │   ├───fig
│       │   ├───log
│       │   │       sim.log
│       │   │       
│       │   └───report
│       ├───20260525_051400
│       │   ├───data
│       │   ├───fig
│       │   ├───log
│       │   │       sim.log
│       │   │       
│       │   └───report
│       ├───20260525_054819
│       │   ├───data
│       │   │       noise_static.csv
│       │   │       
│       │   ├───fig
│       │   │       allan_accel.png
│       │   │       allan_gyro.png
│       │   │       psd_gyro.png
│       │   │       
│       │   ├───log
│       │   │       sim.log
│       │   │       
│       │   └───report
│       ├───20260525_061542
│       │   ├───data
│       │   ├───fig
│       │   ├───log
│       │   │       sim.log
│       │   │       
│       │   └───report
│       ├───20260525_061554
│       │   ├───data
│       │   │       noise_sphere.csv
│       │   │       noise_static.csv
│       │   │       
│       │   ├───fig
│       │   │       allan_accel.png
│       │   │       allan_gyro.png
│       │   │       fft_accel.png
│       │   │       fft_gyro.png
│       │   │       mag_calibration.png
│       │   │       psd_accel.png
│       │   │       psd_gyro.png
│       │   │       psd_mag.png
│       │   │       
│       │   ├───log
│       │   │       sim.log
│       │   │       
│       │   └───report
│       ├───20260525_062119
│       │   ├───data
│       │   ├───fig
│       │   ├───log
│       │   │       sim.log
│       │   │       
│       │   └───report
│       ├───20260525_062239
│       │   ├───data
│       │   ├───fig
│       │   ├───log
│       │   │       sim.log
│       │   │       
│       │   └───report
│       ├───20260525_062307
│       │   ├───data
│       │   ├───fig
│       │   ├───log
│       │   │       sim.log
│       │   │       
│       │   └───report
│       ├───20260525_062342
│       │   ├───data
│       │   ├───fig
│       │   ├───log
│       │   │       sim.log
│       │   │       
│       │   └───report
│       ├───20260525_062418
│       │   ├───data
│       │   ├───fig
│       │   ├───log
│       │   │       sim.log
│       │   │       
│       │   └───report
│       ├───20260525_062424
│       │   ├───data
│       │   ├───fig
│       │   ├───log
│       │   │       sim.log
│       │   │       
│       │   └───report
│       └───20260525_062437
│           ├───data
│           ├───fig
│           ├───log
│           │       sim.log
│           │       
│           └───report
├───config
│       config.yaml
│       
├───export
├───scripts
│       analysis_noise.py
│       check_allan.py
│       plot_results.py
│       run_compare.py
│       
├───src
│   ├───ahrs
│   │   │   main.py
│   │   │   __init__.py
│   │   │   
│   │   ├───core
│   │   │   │   __init__.py
│   │   │   │   
│   │   │   ├───calibration
│   │   │   │   │   accel_cal.py
│   │   │   │   │   gyro_cal.py
│   │   │   │   │   mag_cal.py
│   │   │   │   │   __init__.py
│   │   │   │   │   
│   │   │   │   └───__pycache__
│   │   │   │           accel_cal.cpython-313.pyc
│   │   │   │           gyro_cal.cpython-313.pyc
│   │   │   │           mag_cal.cpython-313.pyc
│   │   │   │           __init__.cpython-313.pyc
│   │   │   │           
│   │   │   ├───env
│   │   │   │   │   environment.py
│   │   │   │   │   gravity_field.py
│   │   │   │   │   magnetic_field.py
│   │   │   │   │   thermal_field.py
│   │   │   │   │   __init__.py
│   │   │   │   │   
│   │   │   │   └───__pycache__
│   │   │   │           environment.cpython-313.pyc
│   │   │   │           gravity_field.cpython-313.pyc
│   │   │   │           magnetic_field.cpython-313.pyc
│   │   │   │           thermal_field.cpython-313.pyc
│   │   │   │           __init__.cpython-313.pyc
│   │   │   │           
│   │   │   ├───estimator
│   │   │   │   │   base.py
│   │   │   │   │   complementary.py
│   │   │   │   │   ekf.py
│   │   │   │   │   ekf_euler.py
│   │   │   │   │   eskf.py
│   │   │   │   │   madgwick.py
│   │   │   │   │   mahony.py
│   │   │   │   │   __init__.py
│   │   │   │   │   
│   │   │   │   └───__pycache__
│   │   │   │           base.cpython-313.pyc
│   │   │   │           complementary.cpython-313.pyc
│   │   │   │           ekf.cpython-313.pyc
│   │   │   │           ekf_euler.cpython-313.pyc
│   │   │   │           eskf.cpython-313.pyc
│   │   │   │           madgwick.cpython-313.pyc
│   │   │   │           mahony.cpython-313.pyc
│   │   │   │           __init__.cpython-313.pyc
│   │   │   │           
│   │   │   ├───evaluation
│   │   │   │   │   allan.py
│   │   │   │   │   fft.py
│   │   │   │   │   metrics.py
│   │   │   │   │   psd.py
│   │   │   │   │   time.py
│   │   │   │   │   __init__.py
│   │   │   │   │   
│   │   │   │   └───__pycache__
│   │   │   │           allan.cpython-313.pyc
│   │   │   │           metrics.cpython-313.pyc
│   │   │   │           psd.cpython-313.pyc
│   │   │   │           report.cpython-313.pyc
│   │   │   │           __init__.cpython-313.pyc
│   │   │   │           
│   │   │   ├───model
│   │   │   │   │   accelerometer.py
│   │   │   │   │   gyroscope.py
│   │   │   │   │   magnetometer.py
│   │   │   │   │   sensor.py
│   │   │   │   │   __init__.py
│   │   │   │   │   
│   │   │   │   └───__pycache__
│   │   │   │           accelerometer.cpython-313.pyc
│   │   │   │           gyroscope.cpython-313.pyc
│   │   │   │           magnetometer.cpython-313.pyc
│   │   │   │           sensor.cpython-313.pyc
│   │   │   │           __init__.cpython-313.pyc
│   │   │   │           
│   │   │   ├───noise
│   │   │   │   │   colored.py
│   │   │   │   │   profile.py
│   │   │   │   │   white.py
│   │   │   │   │   __init__.py
│   │   │   │   │   
│   │   │   │   └───__pycache__
│   │   │   │           colored.cpython-313.pyc
│   │   │   │           profile.cpython-313.pyc
│   │   │   │           white.cpython-313.pyc
│   │   │   │           __init__.cpython-313.pyc
│   │   │   │           
│   │   │   ├───trajectory
│   │   │   │   │   base.py
│   │   │   │   │   circular.py
│   │   │   │   │   figure8.py
│   │   │   │   │   sinusoidal.py
│   │   │   │   │   spherical.py
│   │   │   │   │   static.py
│   │   │   │   │   __init__.py
│   │   │   │   │   
│   │   │   │   └───__pycache__
│   │   │   │           base.cpython-313.pyc
│   │   │   │           circular.cpython-313.pyc
│   │   │   │           figure8.cpython-313.pyc
│   │   │   │           sinusoidal.cpython-313.pyc
│   │   │   │           spherical.cpython-313.pyc
│   │   │   │           static.cpython-313.pyc
│   │   │   │           __init__.cpython-313.pyc
│   │   │   │           
│   │   │   └───__pycache__
│   │   │           __init__.cpython-313.pyc
│   │   │           
│   │   ├───pipeline
│   │   │   │   builder.py
│   │   │   │   runner.py
│   │   │   │   __init__.py
│   │   │   │   
│   │   │   ├───stage
│   │   │   │   │   stage_characterization.py
│   │   │   │   │   stage_estimation.py
│   │   │   │   │   stage_export.py
│   │   │   │   │   stage_report.py
│   │   │   │   │   
│   │   │   │   └───__pycache__
│   │   │   │           stage_characterization.cpython-313.pyc
│   │   │   │           stage_estimation.cpython-313.pyc
│   │   │   │           stage_report.cpython-313.pyc
│   │   │   │           
│   │   │   └───__pycache__
│   │   │           builder.cpython-313.pyc
│   │   │           runner.cpython-313.pyc
│   │   │           __init__.cpython-313.pyc
│   │   │           
│   │   ├───utils
│   │   │   │   configger.py
│   │   │   │   exporter.py
│   │   │   │   logger.py
│   │   │   │   plotter.py
│   │   │   │   reporter.py
│   │   │   │   transforms.py
│   │   │   │   __init__.py
│   │   │   │   
│   │   │   └───__pycache__
│   │   │           configger.cpython-313.pyc
│   │   │           exporter.cpython-313.pyc
│   │   │           logger.cpython-313.pyc
│   │   │           plotter.cpython-313.pyc
│   │   │           reporter.cpython-313.pyc
│   │   │           transforms.cpython-313.pyc
│   │   │           __init__.cpython-313.pyc
│   │   │           
│   │   └───__pycache__
│   │           base.cpython-313.pyc
│   │           main.cpython-313.pyc
│   │           runner.cpython-313.pyc
│   │           simulation.cpython-313.pyc
│   │           __init__.cpython-313.pyc
│   │           
│   └───ahrs.egg-info
│           dependency_links.txt
│           entry_points.txt
│           PKG-INFO
│           requires.txt
│           SOURCES.txt
│           top_level.txt
│           
└───tests
        __init__.p
```

---

## 좌표계 및 컨벤션

| 항목 | 컨벤션 |
|---|---|
| 좌표계 | NED (North-East-Down) |
| 쿼터니언 | scalar-last `[x, y, z, w]`, 항등원 = `[0, 0, 0, 1]` |
| Euler 순서 | ZYX (yaw → pitch → roll), 항공 표준 |
| 내부 단위 | rad (출력·CSV만 deg 변환) |
| 중력 방향 | NED `[0, 0, +g]` (Down = +Z) |
| 자기장 | NED `[North, East, Down]` [μT] |
| DCM 관계 | `v_world = R · v_body` |
| 가속도계 모델 | specific force = `-gravity + linear_accel` (body frame) |
