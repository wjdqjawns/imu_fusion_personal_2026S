"""
File Name: fft.py
Author: Beomjun Chung
Updated: 2026-06-02

Description:
    CSV 파일을 읽어 FFT(Fast Fourier Transform) 시각화.
    센서 raw값과 필터 출력의 주파수 성분을 분석.

Usage:
    python fft.py <csv_path>
    python fft.py                        # 파일 탐색기로 선택
"""

import sys
from pathlib import Path

import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec

# =========================================================
# 그룹 정의
# =========================================================
PLOT_GROUPS = [
    {
        "title": "Accelerometer FFT",
        "ylabel": "Amplitude",
        "cols": ["ax", "ay", "az"],
        "colors": ["#FF4444", "#44CC44", "#4488FF"],
    },
    {
        "title": "Gyroscope FFT",
        "ylabel": "Amplitude",
        "cols": ["gx", "gy", "gz"],
        "colors": ["#FF4444", "#44CC44", "#4488FF"],
    },
    {
        "title": "Magnetometer FFT",
        "ylabel": "Amplitude",
        "cols": ["mx", "my", "mz"],
        "colors": ["#FF4444", "#44CC44", "#4488FF"],
    },
    {
        "title": "Roll FFT",
        "ylabel": "Amplitude",
        "cols": ["roll_cf", "roll_kf"],
        "colors": ["#378ADD", "#E24B4A"],
    },
    {
        "title": "Pitch FFT",
        "ylabel": "Amplitude",
        "cols": ["pitch_cf", "pitch_kf"],
        "colors": ["#378ADD", "#E24B4A"],
    },
    {
        "title": "Yaw FFT",
        "ylabel": "Amplitude",
        "cols": ["yaw_cf", "yaw_kf"],
        "colors": ["#378ADD", "#E24B4A"],
    },
]


# =========================================================
# FFT 계산
# =========================================================
def compute_fft(signal: np.ndarray, fs: float):
    """
    단측 FFT 계산.
    Returns: freq [Hz], amplitude (normalized)
    """
    n    = len(signal)
    sig  = signal - np.mean(signal)          # DC 제거
    win  = np.hanning(n)                     # Hanning window
    fft  = np.fft.rfft(sig * win)
    freq = np.fft.rfftfreq(n, d=1.0 / fs)
    amp  = (2.0 / n) * np.abs(fft)          # 단측 정규화
    return freq, amp

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
        fs = 100.0   # 기본 샘플링 주파수
    print(f"[Fs] {fs:.2f} Hz  ({len(df)} samples)")
    return df, fs


# =========================================================
# 그룹별 FFT subplot 그리기
# =========================================================
def plot_fft(df: pd.DataFrame, fs: float, csv_path: Path):
    n_groups = len(PLOT_GROUPS)

    fig = plt.figure(figsize=(16, 3 * n_groups), facecolor="#1a1a1a")
    fig.suptitle(
        f"FFT Analysis  —  {csv_path.name}  (Fs = {fs:.1f} Hz)",
        color="white", fontsize=12, fontweight="bold",
    )

    gs = gridspec.GridSpec(
        n_groups, 1,
        hspace=0.55,
        left=0.07, right=0.97,
        top=0.96, bottom=0.04,
    )

    axes = []
    for i, grp in enumerate(PLOT_GROUPS):
        ax = fig.add_subplot(gs[i])
        if i > 0:
            ax.sharex(axes[0])

        for col, color in zip(grp["cols"], grp["colors"]):
            if col not in df.columns:
                continue
            freq, amp = compute_fft(df[col].values, fs)
            label = col.split("_")[-1].upper() if "_" in col else col
            ax.plot(freq, amp,
                    color=color, linewidth=0.8, label=label, alpha=0.9)

        ax.set_title(grp["title"], color="#aaaaaa", fontsize=9,
                     loc="left", pad=3)
        ax.set_ylabel(grp["ylabel"], color="white", fontsize=8)
        ax.tick_params(colors="white", labelsize=7)
        ax.spines[["bottom", "top", "left", "right"]].set_color("#555")
        ax.set_facecolor("#111111")
        ax.grid(True, color="#2a2a2a", linewidth=0.5)
        ax.legend(loc="upper right", fontsize=7,
                  framealpha=0.3, labelcolor="white", facecolor="#222")
        ax.set_xlim(left=0)

        if i < n_groups - 1:
            ax.tick_params(labelbottom=False)
        else:
            ax.set_xlabel("Frequency [Hz]", color="white", fontsize=8)

        axes.append(ax)

    # ── 저장 ─────────────────────────────────────────────
    out_dir = csv_path.parent.parent / "fig"
    out_dir.mkdir(parents=True, exist_ok=True)
    out_path = out_dir / f"{csv_path.stem}_fft.png"
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
    plot_fft(df, fs, csv_path)


if __name__ == "__main__":
    main()