"""
File Name: psd.py
Author: Beomjun Chung
Updated: 2026-06-02

Description:
    CSV 파일을 읽어 PSD(Power Spectral Density) 시각화.
    Welch 방법으로 추정. y축은 dB 스케일.

Usage:
    python psd.py <csv_path>
    python psd.py                        # 파일 탐색기로 선택
"""

import sys
from pathlib import Path

import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec
from scipy.signal import welch


# =========================================================
# 그룹 정의
# =========================================================
PLOT_GROUPS = [
    {
        "title": "Accelerometer PSD",
        "ylabel": "PSD [dB]",
        "cols": ["ax", "ay", "az"],
        "colors": ["#FF4444", "#44CC44", "#4488FF"],
    },
    {
        "title": "Gyroscope PSD",
        "ylabel": "PSD [dB]",
        "cols": ["gx", "gy", "gz"],
        "colors": ["#FF4444", "#44CC44", "#4488FF"],
    },
    {
        "title": "Magnetometer PSD",
        "ylabel": "PSD [dB]",
        "cols": ["mx", "my", "mz"],
        "colors": ["#FF4444", "#44CC44", "#4488FF"],
    },
    {
        "title": "Roll PSD",
        "ylabel": "PSD [dB]",
        "cols": ["roll_cf", "roll_kf"],
        "colors": ["#378ADD", "#E24B4A"],
    },
    {
        "title": "Pitch PSD",
        "ylabel": "PSD [dB]",
        "cols": ["pitch_cf", "pitch_kf"],
        "colors": ["#378ADD", "#E24B4A"],
    },
    {
        "title": "Yaw PSD",
        "ylabel": "PSD [dB]",
        "cols": ["yaw_cf", "yaw_kf"],
        "colors": ["#378ADD", "#E24B4A"],
    },
]

# Welch 파라미터
NPERSEG    = 512    # segment 길이 (2의 거듭제곱 권장)
OVERLAP    = 0.5    # 50% overlap


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
    return df, fs


# =========================================================
# PSD subplot 그리기
# =========================================================
def plot_psd(df: pd.DataFrame, fs: float, csv_path: Path):
    n_groups = len(PLOT_GROUPS)
    nperseg  = min(NPERSEG, len(df) // 2)

    fig = plt.figure(figsize=(16, 3 * n_groups), facecolor="#1a1a1a")
    fig.suptitle(
        f"PSD Analysis (Welch)  —  {csv_path.name}  (Fs = {fs:.1f} Hz)",
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
            sig  = df[col].values - np.mean(df[col].values)
            freq, pxx = welch(
                sig, fs=fs,
                nperseg=nperseg,
                noverlap=int(nperseg * OVERLAP),
                window="hann",
            )
            pxx_db = 10 * np.log10(pxx + 1e-12)   # dB 변환, 0 방지
            label  = col.split("_")[-1].upper() if "_" in col else col
            ax.plot(freq, pxx_db,
                    color=color, linewidth=0.9, label=label, alpha=0.9)

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
    out_path = out_dir / f"{csv_path.stem}_psd.png"
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
    plot_psd(df, fs, csv_path)


if __name__ == "__main__":
    main()