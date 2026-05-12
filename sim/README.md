# prj_ahrs Python scaffold

Windows-friendly AHRS simulation scaffold for comparing:
- Mahony filter
- Madgwick filter
- EKF-based AHRS

## Quick start

```powershell
cd C:\Users\cbj\Desktop\prj_ahrs
python -m venv .venv
.\.venv\Scripts\activate
pip install -r requirements.txt
python scripts\run_compare.py
python scripts\plot_results.py
```

Outputs are saved under `data/`.

## What this does

- generates a synthetic rigid-body trajectory
- simulates gyro / accel / mag measurements with bias and noise
- runs three AHRS estimators on the same data
- compares roll / pitch / yaw error against ground truth

## Next steps

1. Replace `src/simulation.py` sensor model with your MPU9250 real log format
2. Keep the same estimator interface
3. Add Allan variance / temperature bias analysis scripts
4. Reuse plotting scripts for real data
