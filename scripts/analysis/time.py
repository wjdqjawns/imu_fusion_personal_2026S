"""
File Name: time.py
Author: Beomjun Chung
Updated: 2026-06-02

Description:
    CSV 파일을 읽어 시간축 시각화.
    센서 raw값(accel/gyro/mag)과 필터 출력(roll/pitch/yaw)을
    그룹별 subplot으로 표현.

Usage:
    python time.py <csv_path>
    python time.py                        # 파일 탐색기로 선택
"""

import sys
from pathlib import Path

import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec


# =========================================================
# 그룹 정의 — HEADER 순서 기준
# =========================================================
PLOT_GROUPS = [
    {
        "title": "Accelerometer Raw",
        "ylabel": "[m/s²]",
        "cols": ["ax", "ay", "az"],
        "colors": ["#FF4444", "#44CC44", "#4488FF"],
    },
    {
        "title": "Gyroscope Raw",
        "ylabel": "[deg/s]",
        "cols": ["gx", "gy", "gz"],
        "colors": ["#FF4444", "#44CC44", "#4488FF"],
    },
    {
        "title": "Magnetometer Raw",
        "ylabel": "[uT]",
        "cols": ["mx", "my", "mz"],
        "colors": ["#FF4444", "#44CC44", "#4488FF"],
    },
    {
        "title": "Roll",
        "ylabel": "[deg]",
        "cols": ["roll_cf", "roll_kf"],
        "colors": ["#378ADD", "#E24B4A"],
    },
    {
        "title": "Pitch",
        "ylabel": "[deg]",
        "cols": ["pitch_cf", "pitch_kf"],
        "colors": ["#378ADD", "#E24B4A"],
    },
    {
        "title": "Yaw",
        "ylabel": "[deg]",
        "cols": ["yaw_cf", "yaw_kf"],
        "colors": ["#378ADD", "#E24B4A"],
    },
]


# =========================================================
# CSV 로드
# =========================================================
def load_csv(path: Path) -> pd.DataFrame:
    df = pd.read_csv(path)
    # time_ms를 초 단위로 변환
    if "time_ms" in df.columns:
        df["time_s"] = df["time_ms"] / 1000.0
    return df


# =========================================================
# 그룹별 subplot 그리기
# =========================================================
def plot_time(df: pd.DataFrame, csv_path: Path):
    n_groups = len(PLOT_GROUPS)
    t = df["time_s"].values if "time_s" in df.columns else df.index.values

    fig = plt.figure(figsize=(16, 3 * n_groups), facecolor="#1a1a1a")
    fig.suptitle(
        f"Time Domain Analysis  —  {csv_path.name}",
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
            # 라벨: 컬럼명에서 _cf/_kf 접미사 추출해서 표시
            label = col.split("_")[-1].upper() if "_" in col else col
            ax.plot(t, df[col].values,
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

        # x축 라벨은 마지막 subplot에만
        if i < n_groups - 1:
            ax.tick_params(labelbottom=False)
        else:
            ax.set_xlabel("Time [s]", color="white", fontsize=8)

        axes.append(ax)

    # ── 통계 요약 출력 ────────────────────────────────────
    print(f"\n{'='*60}")
    print(f"File : {csv_path.name}")
    print(f"Rows : {len(df)}")
    if "time_s" in df.columns:
        duration = df["time_s"].iloc[-1] - df["time_s"].iloc[0]
        fs = len(df) / duration if duration > 0 else 0
        print(f"Duration : {duration:.2f} s")
        print(f"Avg Fs   : {fs:.1f} Hz")
    print(f"{'='*60}")

    numeric_cols = [c for grp in PLOT_GROUPS for c in grp["cols"] if c in df.columns]
    stats = df[numeric_cols].describe().loc[["mean", "std", "min", "max"]]
    print(stats.to_string())
    print(f"{'='*60}\n")

    # ── 저장 ─────────────────────────────────────────────
    out_dir = csv_path.parent.parent / "fig"
    out_dir.mkdir(parents=True, exist_ok=True)
    out_path = out_dir / f"{csv_path.stem}_time.png"
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
        # 파일 탐색기로 선택
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
    df = load_csv(csv_path)
    print(f"[Loaded]  {len(df)} rows, {len(df.columns)} cols")

    plot_time(df, csv_path)


if __name__ == "__main__":
    main()