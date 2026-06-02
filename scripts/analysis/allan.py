"""
File Name: allan.py
Author: Beomjun Chung
Updated: 2026-06-02

Description:
    CSV 파일을 읽어 Allan Deviation 시각화.
    정지 상태 데이터에서 센서 노이즈 특성 분석에 사용.
    - Angle Random Walk (ARW): 기울기 -1/2
    - Bias Instability    : 기울기  0
    - Rate Random Walk    : 기울기 +1/2

    Allan Deviation이 의미있으려면 정지 상태 데이터가 필요.

Usage:
    python allan.py <csv_path>
    python allan.py                        # 파일 탐색기로 선택

Install:
    pip install allantools
"""

import sys
from pathlib import Path

import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec

try:
    import allantools
    HAS_ALLANTOOLS = True
except ImportError:
    HAS_ALLANTOOLS = False
    print("[Warning] allantools 미설치 → pip install allantools")
    print("          수동 계산 방법으로 대체합니다.\n")


# =========================================================
# 그룹 정의 (Allan은 raw 센서 + 필터 출력)
# =========================================================
PLOT_GROUPS = [
    {
        "title": "Accelerometer Allan Deviation",
        "ylabel": "ADEV",
        "unit": "m/s²",
        "cols": ["ax", "ay", "az"],
        "colors": ["#FF4444", "#44CC44", "#4488FF"],
    },
    {
        "title": "Gyroscope Allan Deviation",
        "ylabel": "ADEV",
        "unit": "deg/s",
        "cols": ["gx", "gy", "gz"],
        "colors": ["#FF4444", "#44CC44", "#4488FF"],
    },
    {
        "title": "Roll Allan Deviation",
        "ylabel": "ADEV",
        "unit": "deg",
        "cols": ["roll_cf", "roll_kf"],
        "colors": ["#378ADD", "#E24B4A"],
    },
    {
        "title": "Pitch Allan Deviation",
        "ylabel": "ADEV",
        "unit": "deg",
        "cols": ["pitch_cf", "pitch_kf"],
        "colors": ["#378ADD", "#E24B4A"],
    },
    {
        "title": "Yaw Allan Deviation",
        "ylabel": "ADEV",
        "unit": "deg",
        "cols": ["yaw_cf", "yaw_kf"],
        "colors": ["#378ADD", "#E24B4A"],
    },
]


# =========================================================
# Allan Deviation 계산
# =========================================================
def compute_adev(data: np.ndarray, fs: float):
    """
    allantools 있으면 사용, 없으면 수동 계산.
    Returns: tau [s], adev
    """
    if HAS_ALLANTOOLS:
        taus = np.logspace(
            np.log10(1.0 / fs),
            np.log10(len(data) / (10 * fs)),
            100,
        )
        tau_out, adev, _, _ = allantools.oadev(
            data, rate=fs, data_type="freq", taus=taus
        )
        return tau_out, adev
    else:
        # 수동 overlapping Allan deviation
        n   = len(data)
        fs_ = float(fs)
        max_m = int(n / 10)
        ms    = np.unique(np.logspace(0, np.log10(max_m), 200).astype(int))
        ms    = ms[ms >= 1]
        taus, adevs = [], []
        for m in ms:
            tau = m / fs_
            # Overlapping Allan
            k    = n - 2 * m
            if k <= 0:
                continue
            s = 0.0
            for j in range(k):
                diff = (np.sum(data[j + m:j + 2 * m])
                        - np.sum(data[j:j + m])) / m
                s += diff ** 2
            adev = np.sqrt(s / (2 * k))
            taus.append(tau)
            adevs.append(adev)
        return np.array(taus), np.array(adevs)


# =========================================================
# 기울기 참조선 그리기
# =========================================================
def _draw_slope_lines(ax, tau, adev):
    """ARW(-1/2), Bias(0), RRW(+1/2) 참조선"""
    tau_mid = np.exp(np.mean(np.log(tau)))
    adev_mid = np.exp(np.mean(np.log(adev + 1e-20)))

    slopes = [
        (-0.5, "--", "#888888", "ARW (-½)"),
        ( 0.0, ":",  "#666666", "Bias (0)"),
        ( 0.5, "-.", "#666666", "RRW (+½)"),
    ]
    for slope, ls, color, label in slopes:
        y_ref = adev_mid * (tau / tau_mid) ** slope
        ax.plot(tau, y_ref, linestyle=ls, color=color,
                linewidth=0.7, alpha=0.5, label=label)


# =========================================================
# CSV 로드
# =========================================================
def load_csv(path: Path) -> tuple[pd.DataFrame, float]:
    df = pd.read_csv(path)
    if "time_ms" in df.columns:
        df["time_s"] = df["time_ms"] / 1000.0
        duration = df["time_s"].iloc[-1] - df["time_s"].iloc[0]
        fs = len(df) / duration if duration > 0 else 100.0
    else:
        fs = 100.0
    print(f"[Fs] {fs:.2f} Hz  ({len(df)} samples)")
    if len(df) < 1000:
        print("[Warning] Allan Deviation은 정지 상태의 긴 데이터(권장 10분+)가 필요합니다.")
    return df, fs


# =========================================================
# Allan subplot 그리기
# =========================================================
def plot_allan(df: pd.DataFrame, fs: float, csv_path: Path):
    n_groups = len(PLOT_GROUPS)

    fig = plt.figure(figsize=(14, 3.2 * n_groups), facecolor="#1a1a1a")
    fig.suptitle(
        f"Allan Deviation  —  {csv_path.name}  (Fs = {fs:.1f} Hz)",
        color="white", fontsize=12, fontweight="bold",
    )

    gs = gridspec.GridSpec(
        n_groups, 1,
        hspace=0.60,
        left=0.08, right=0.97,
        top=0.96, bottom=0.04,
    )

    for i, grp in enumerate(PLOT_GROUPS):
        ax = fig.add_subplot(gs[i])

        first_tau = None
        for col, color in zip(grp["cols"], grp["colors"]):
            if col not in df.columns:
                continue
            print(f"  Computing ADEV: {col} ...", end=" ", flush=True)
            tau, adev = compute_adev(df[col].values, fs)
            print("done")
            label = col.split("_")[-1].upper() if "_" in col else col
            ax.loglog(tau, adev, color=color, linewidth=1.0,
                      label=label, alpha=0.9)
            if first_tau is None:
                first_tau = tau
                first_adev = adev

        if first_tau is not None:
            _draw_slope_lines(ax, first_tau, first_adev)

        ylabel = f"{grp['ylabel']} [{grp['unit']}]"
        ax.set_title(grp["title"], color="#aaaaaa", fontsize=9,
                     loc="left", pad=3)
        ax.set_ylabel(ylabel, color="white", fontsize=8)
        ax.tick_params(colors="white", labelsize=7,
                       which="both")
        ax.spines[["bottom", "top", "left", "right"]].set_color("#555")
        ax.set_facecolor("#111111")
        ax.grid(True, which="both", color="#2a2a2a", linewidth=0.5)
        ax.legend(loc="upper right", fontsize=7,
                  framealpha=0.3, labelcolor="white", facecolor="#222")

        if i < n_groups - 1:
            ax.tick_params(labelbottom=False)
        else:
            ax.set_xlabel("Averaging Time τ [s]", color="white", fontsize=8)

    # ── 저장 ─────────────────────────────────────────────
    out_dir = csv_path.parent.parent / "fig"
    out_dir.mkdir(parents=True, exist_ok=True)
    out_path = out_dir / f"{csv_path.stem}_allan.png"
    fig.savefig(out_path, dpi=150, bbox_inches="tight", facecolor="#1a1a1a")
    print(f"[Saved] {out_path}")

    plt.show()


# =========================================================
# Main
# =========================================================
def main():
    if len(sys.argv) >= 2:
        csv_path = Path(sys.argv[1])
    else:
        import tkinter as tk
        from tkinter import filedialog
        root = tk.Tk()
        root.withdraw()
        p = filedialog.askopenfilename(
            title="CSV 파일 선택",
            filetypes=[("CSV files", "*.csv"), ("All files", "*.*")],
        )
        if not p:
            print("파일을 선택하지 않았습니다.")
            return
        csv_path = Path(p)

    if not csv_path.exists():
        print(f"[Error] 파일 없음: {csv_path}")
        return

    print(f"[Loading] {csv_path}")
    df, fs = load_csv(csv_path)
    plot_allan(df, fs, csv_path)


if __name__ == "__main__":
    main()