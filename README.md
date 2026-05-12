# imu_fusion

MPU-9250을 이용한 AHRS(Attitude and Heading Reference System) 구현 프로젝트입니다.  
센서 캘리브레이션부터 EKF 기반 자세 추정까지 전 과정을 다룹니다.

---

## 개요

MEMS 기반 IMU(MPU-9250)의 노이즈 특성 분석, 캘리브레이션, 그리고 자세 추정 알고리즘 설계 및 검증을 목적으로 합니다.

- **센서**: MPU-9250 (가속도계 + 자이로스코프 + AK8963 지자기계)
- **플랫폼**: Arduino + Python 시뮬레이션
- **알고리즘**: Complementary Filter, Mahony Filter, Madgwick Filter, EKF

---

## 프로젝트 구조

```
imu_fusion/
├── sim/                        # Python 시뮬레이션
│   ├── src/
│   │   ├── filters/            # 필터 구현 (CF, Mahony, Madgwick, EKF)
│   │   └── utils/              # Allan Variance, PSD, 유틸리티
│   ├── scripts/                # 실행 스크립트
│   ├── config/                 # 설정 파일 (잡음 파라미터, 캘리브레이션 값)
│   └── data/                   # 측정 데이터 (정적, 동적, 캘리브레이션)
│
├── src/imu_fusion/             # Arduino 펌웨어 (PlatformIO)
│   ├── src/                    # 메인 소스
│   ├── lib/
│   │   ├── imu/                # MPU-9250 드라이버 래퍼
│   │   └── estimation/         # EKF 임베디드 구현
│   └── include/                # 헤더 파일
│
├── doc/
│   └── report/                 # 보고서 (LaTeX)
│
├── data/                       # 원시 측정 데이터
├── export/                     # 출력 결과 (그래프, 로그)
├── scripts/                    # 분석 자동화 스크립트
└── tests/                      # 단위 테스트
```

---

## 구현 내용

### 1. 센서 노이즈 모델링
- Allan Variance(AVAR) 분석을 통한 ARW, Bias Instability, RRW 계수 추출
- Power Spectral Density(PSD) 분석

### 2. 센서 캘리브레이션
| 센서 | 방법 |
|---|---|
| 가속도계 | 6-위치 정적 캘리브레이션 (바이어스, 스케일 팩터, 축 정렬) |
| 자이로스코프 | 정적 바이어스 추정, Allan Variance 기반 잡음 파라미터 추출 |
| 지자기계 | Hard Iron 보정 + Soft Iron 보정 (Ellipsoid Fitting) |

### 3. AHRS 알고리즘

| 알고리즘 | 자세 표현 | 바이어스 추정 | 특징 |
|---|---|---|---|
| Complementary Filter | 오일러각 | ✗ | 구현 단순, 고정 α |
| Mahony Filter | 쿼터니언 | ✓ (PI) | 임베디드 최적, kp/ki 튜닝 |
| Madgwick Filter | 쿼터니언 | ✗ | 단일 파라미터 β |
| EKF | 쿼터니언 | ✓ | 최적 추정, Allan Variance 기반 Q 설계 |

EKF 상태 벡터: `x = [q₀, q₁, q₂, q₃, bgx, bgy, bgz]ᵀ ∈ ℝ⁷`

---

## 시작하기

### Python 시뮬레이션

**요구 사항**

```
Python 3.8+
numpy
scipy
matplotlib
```

**설치 및 실행**

```bash
cd sim
python -m venv .venv
source .venv/bin/activate        # Windows: .venv\Scripts\activate
pip install numpy scipy matplotlib

# 시뮬레이션 실행 (전체 필터 비교)
python scripts/main.py --mode sim --filter all --plot

# Allan Variance 분석
python scripts/main.py --mode allan --input data/gyro_static.csv

# 캘리브레이션
python scripts/main.py --mode calib --sensor accel --input data/accel_6pos.csv
python scripts/main.py --mode calib --sensor mag   --input data/mag_rotation.csv
```

### Arduino 펌웨어

**요구 사항**: PlatformIO

```bash
cd src/imu_fusion
pio run --target upload        # 빌드 및 업로드
pio device monitor             # 시리얼 모니터 (115200 baud)
```

### 실제 데이터 수집

```bash
# 정적 데이터 수집 (Allan Variance용, 최소 2시간 권장)
python sim/scripts/collect.py --port COM3 --duration 7200 --out data/gyro_static.csv

# 동적 실험 데이터 수집
python sim/scripts/collect.py --port COM3 --duration 120 --out data/dynamic.csv
```

---

## EKF 설계 요약

```
공정 잡음 공분산 Q:
  - Qq (쿼터니언): ARW 계수 N으로부터 결정
  - Qb (바이어스): RRW 계수 K로부터 결정

측정 잡음 공분산 R:
  - Ra: 가속도계 캘리브레이션 잔차 분산
  - Rm: 지자기계 캘리브레이션 잔차 분산

Update 조건 (동적 가속도 감지):
  - 가속도계: | ‖ã‖ - g | < 0.5 m/s²  →  Update 수행
  - 조건 미충족 시 Prediction만 수행
```

---

## 참고 자료

- MPU-9250 Product Specification, InvenSense
- Madgwick, S. O. H. (2010). *An efficient orientation filter for inertial and inertial/magnetic sensor arrays.*
- Mahony, R. et al. (2008). Nonlinear complementary filters on the special orthogonal group. *IEEE TAC.*
- IEEE Std 952-1997: *Specification Format Guide for Single-Axis Interferometric Fiber Optic Gyros* (Allan Variance 정의)

---

## 과목 정보

| 항목 | 내용 |
|---|---|
| 과목 | MECH407 Introduction to Mobile Systems |
| 담당 교수 | SeongMin Lee |
| 유형 | Term Project 01 — IMU State Estimation |
| 제출일 | 2026년 5월 13일 |
| 학생 | 정범준 / 202211178 |