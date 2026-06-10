"""
File Name: serial_plotter.py
Author: Beomjun Chung
Updated: 2026-06-02

Description:
    시리얼 데이터를 받아 그룹별 subplot으로 실시간 시각화.
    3D 없이 순수 시계열 그래프만.

    레이아웃:
    ┌──────────────────────────────────────────┐
    │  Accel Raw   (ax, ay, az)                │
    │  Gyro Raw    (gx, gy, gz)                │
    │  Mag Raw     (mx, my, mz)                │
    │  Roll        (Gyro / CF / EKF / Mdg / Mhn) │
    │  Pitch       (Gyro / CF / EKF / Mdg / Mhn) │
    │  Yaw         (Gyro / CF / EKF / Mdg / Mhn) │
    │  Error Norm  (‖CF−EKF‖)                  │
    └──────────────────────────────────────────┘

    s 키: snapshot PNG 저장
"""

import serial
import csv
import threading
import queue
from collections import deque
from datetime import datetime
from pathlib import Path

import numpy as np
import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec
import matplotlib.animation as animation

# =========================================================
# Config
# =========================================================
MODE_DATA   = "MOCK"   # "REAL" or "MOCK"

SERIAL_PORT = "COM8"
SERIAL_BAUD = 921600

QUEUE_SIZE       = 10000
PLOT_BUFFER_SIZE = 1000
PLOT_UPDATE_MS   = 100
SNAPSHOT_KEY     = "s"

EXPORT_ROOT = Path(__file__).parent.parent / "export"

HEADER = [
    "t_ms",
    "ax", "ay", "az",
    "gx", "gy", "gz",
    "mx", "my", "mz",
    "roll_gyro_prop", "pitch_gyro_prop", "yaw_gyro_prop",
    "roll_cf",        "pitch_cf",        "yaw_cf",
    "roll_ekf",       "pitch_ekf",       "yaw_ekf",
    "roll_madgwick",  "pitch_madgwick",  "yaw_madgwick",
    "roll_mahony",    "pitch_mahony",    "yaw_mahony",
]
N_COLS     = len(HEADER)   # 25
PLOT_COL_X = 0

ROLL_GYRO_COL  = 10; PITCH_GYRO_COL  = 11; YAW_GYRO_COL  = 12
ROLL_CF_COL    = 13; PITCH_CF_COL    = 14; YAW_CF_COL    = 15
ROLL_EKF_COL   = 16; PITCH_EKF_COL   = 17; YAW_EKF_COL   = 18
ROLL_MDW_COL   = 19; PITCH_MDW_COL   = 20; YAW_MDW_COL   = 21
ROLL_MHN_COL   = 22; PITCH_MHN_COL   = 23; YAW_MHN_COL   = 24

FILTER_COLORS = {
    "Gyro":     "#AAAAAA",
    "CF":       "#378ADD",
    "EKF":      "#E24B4A",
    "Madgwick": "#44AA66",
    "Mahony":   "#EF9F27",
}
FILTER_LS = {
    "Gyro": ":",
    "CF": "-",
    "EKF": "--",
    "Madgwick": "-.",
    "Mahony": (0, (3, 1, 1, 1)),
}

# =========================================================
# 그룹 정의
# =========================================================
PLOT_GROUPS = [
    {
        "title": "Accelerometer",
        "ylabel": "[m/s²]",
        "channels": [
            {"col": 1, "label": "ax", "color": "#FF4444", "ls": "-"},
            {"col": 2, "label": "ay", "color": "#44CC44", "ls": "-"},
            {"col": 3, "label": "az", "color": "#4488FF", "ls": "-"},
        ],
    },
    {
        "title": "Gyroscope",
        "ylabel": "[deg/s]",
        "channels": [
            {"col": 4, "label": "gx", "color": "#FF4444", "ls": "-"},
            {"col": 5, "label": "gy", "color": "#44CC44", "ls": "-"},
            {"col": 6, "label": "gz", "color": "#4488FF", "ls": "-"},
        ],
    },
    {
        "title": "Magnetometer",
        "ylabel": "[uT]",
        "channels": [
            {"col": 7, "label": "mx", "color": "#FF4444", "ls": "-"},
            {"col": 8, "label": "my", "color": "#44CC44", "ls": "-"},
            {"col": 9, "label": "mz", "color": "#4488FF", "ls": "-"},
        ],
    },
    {
        "title": "Roll",
        "ylabel": "[deg]",
        "channels": [
            {"col": ROLL_GYRO_COL, "label": "Gyro",     "color": FILTER_COLORS["Gyro"],     "ls": FILTER_LS["Gyro"]},
            {"col": ROLL_CF_COL,   "label": "CF",       "color": FILTER_COLORS["CF"],       "ls": FILTER_LS["CF"]},
            {"col": ROLL_EKF_COL,  "label": "EKF",      "color": FILTER_COLORS["EKF"],      "ls": FILTER_LS["EKF"]},
            {"col": ROLL_MDW_COL,  "label": "Madgwick", "color": FILTER_COLORS["Madgwick"], "ls": FILTER_LS["Madgwick"]},
            {"col": ROLL_MHN_COL,  "label": "Mahony",   "color": FILTER_COLORS["Mahony"],   "ls": FILTER_LS["Mahony"]},
        ],
    },
    {
        "title": "Pitch",
        "ylabel": "[deg]",
        "channels": [
            {"col": PITCH_GYRO_COL, "label": "Gyro",     "color": FILTER_COLORS["Gyro"],     "ls": FILTER_LS["Gyro"]},
            {"col": PITCH_CF_COL,   "label": "CF",       "color": FILTER_COLORS["CF"],       "ls": FILTER_LS["CF"]},
            {"col": PITCH_EKF_COL,  "label": "EKF",      "color": FILTER_COLORS["EKF"],      "ls": FILTER_LS["EKF"]},
            {"col": PITCH_MDW_COL,  "label": "Madgwick", "color": FILTER_COLORS["Madgwick"], "ls": FILTER_LS["Madgwick"]},
            {"col": PITCH_MHN_COL,  "label": "Mahony",   "color": FILTER_COLORS["Mahony"],   "ls": FILTER_LS["Mahony"]},
        ],
    },
    {
        "title": "Yaw",
        "ylabel": "[deg]",
        "channels": [
            {"col": YAW_GYRO_COL, "label": "Gyro",     "color": FILTER_COLORS["Gyro"],     "ls": FILTER_LS["Gyro"]},
            {"col": YAW_CF_COL,   "label": "CF",       "color": FILTER_COLORS["CF"],       "ls": FILTER_LS["CF"]},
            {"col": YAW_EKF_COL,  "label": "EKF",      "color": FILTER_COLORS["EKF"],      "ls": FILTER_LS["EKF"]},
            {"col": YAW_MDW_COL,  "label": "Madgwick", "color": FILTER_COLORS["Madgwick"], "ls": FILTER_LS["Madgwick"]},
            {"col": YAW_MHN_COL,  "label": "Mahony",   "color": FILTER_COLORS["Mahony"],   "ls": FILTER_LS["Mahony"]},
        ],
    },
]

# =========================================================
# Session 폴더
# =========================================================
session_ts  = datetime.now().strftime("%Y_%m_%d_%H%M%S")
session_dir = EXPORT_ROOT / session_ts
data_dir    = session_dir / "data"
fig_dir     = session_dir / "fig"

for d in (data_dir, fig_dir):
    d.mkdir(parents=True, exist_ok=True)

csv_path = data_dir / f"imu_{session_ts}.csv"
print(f"[Logger] Saving → {csv_path}")

# =========================================================
# Globals
# =========================================================
data_queue = queue.Queue(maxsize=QUEUE_SIZE)
plot_buf_x = deque(maxlen=PLOT_BUFFER_SIZE)
running    = True

plot_bufs = [
    [deque(maxlen=PLOT_BUFFER_SIZE) for _ in grp["channels"]]
    for grp in PLOT_GROUPS
]

err_buf = deque(maxlen=PLOT_BUFFER_SIZE)

# =========================================================
# Serial
# =========================================================
if MODE_DATA == "REAL":
    ser = serial.Serial(SERIAL_PORT, SERIAL_BAUD, timeout=1)
elif MODE_DATA == "MOCK":
    from scripts.mock.mock_serial import MockSerial
    ser = MockSerial()

# =========================================================
# Threads
# =========================================================
def read_serial():
    while running:
        try:
            line = ser.readline().decode("utf-8", errors="ignore").strip()
            if line:
                if data_queue.full():
                    data_queue.get_nowait()
                data_queue.put(line)
        except Exception as e:
            print(f"[Reader] {e}")


def write_csv():
    with open(csv_path, "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(HEADER)
        while running or not data_queue.empty():
            try:
                line = data_queue.get(timeout=0.5)
                cols = line.rstrip(",").split(",")   # trailing comma 제거
                if len(cols) != N_COLS:
                    continue
                writer.writerow(cols)
                f.flush()
                try:
                    plot_buf_x.append(float(cols[PLOT_COL_X]))

                    for g_idx, grp in enumerate(PLOT_GROUPS):
                        for c_idx, ch in enumerate(grp["channels"]):
                            plot_bufs[g_idx][c_idx].append(float(cols[ch["col"]]))

                    dr = float(cols[ROLL_CF_COL])  - float(cols[ROLL_EKF_COL])
                    dp = float(cols[PITCH_CF_COL]) - float(cols[PITCH_EKF_COL])
                    dy = float(cols[YAW_CF_COL])   - float(cols[YAW_EKF_COL])
                    err_buf.append(np.sqrt(dr**2 + dp**2 + dy**2))
                except ValueError:
                    pass
            except queue.Empty:
                continue
            except Exception as e:
                print(f"[Writer] {e}")


reader_thread = threading.Thread(target=read_serial, daemon=True)
writer_thread = threading.Thread(target=write_csv,   daemon=True)
reader_thread.start()
writer_thread.start()

# =========================================================
# Figure 레이아웃
# =========================================================
n_groups = len(PLOT_GROUPS) + 1   # +1 for error norm
fig = plt.figure(figsize=(14, 2.2 * n_groups), facecolor="#1a1a1a")
fig.suptitle("IMU Serial Plotter", color="white", fontsize=12, fontweight="bold")

gs = gridspec.GridSpec(
    n_groups, 1,
    hspace=0.50,
    left=0.07, right=0.97,
    top=0.96, bottom=0.03,
)

axes = [fig.add_subplot(gs[i]) for i in range(n_groups)]

for ax in axes[1:]:
    ax.sharex(axes[0])

# ── 그룹 subplot 초기화 ───────────────────────────────────
all_lines = []

for g_idx, (ax, grp) in enumerate(zip(axes[:-1], PLOT_GROUPS)):
    g_lines = []
    for ch in grp["channels"]:
        (lp,) = ax.plot(
            [], [],
            color=ch["color"],
            linestyle=ch["ls"],
            linewidth=1.0,
            label=ch["label"],
        )
        g_lines.append(lp)

    ax.set_title(grp["title"], color="#aaaaaa", fontsize=8, loc="left", pad=2)
    ax.set_ylabel(grp["ylabel"], color="white", fontsize=8)
    ax.tick_params(colors="white", labelsize=7)
    ax.spines[["bottom", "top", "left", "right"]].set_color("#555")
    ax.set_facecolor("#111111")
    ax.grid(True, color="#2a2a2a", linewidth=0.5)
    ax.legend(loc="upper right", fontsize=7,
              framealpha=0.3, labelcolor="white", facecolor="#222",
              ncol=5)
    ax.tick_params(labelbottom=False)
    all_lines.append(g_lines)

# ── Error Norm subplot ────────────────────────────────────
ax_err = axes[-1]
(err_line,) = ax_err.plot([], [], color="#EF9F27", linewidth=1.0, label="‖CF−EKF‖")
ax_err.set_title("Error Norm", color="#aaaaaa", fontsize=8, loc="left", pad=2)
ax_err.set_ylabel("[deg]", color="white", fontsize=8)
ax_err.set_xlabel("Time [ms]", color="white", fontsize=8)
ax_err.tick_params(colors="white", labelsize=7)
ax_err.spines[["bottom", "top", "left", "right"]].set_color("#555")
ax_err.set_facecolor("#111111")
ax_err.grid(True, color="#2a2a2a", linewidth=0.5)
ax_err.legend(loc="upper right", fontsize=7,
              framealpha=0.3, labelcolor="white", facecolor="#222")
ax_err.set_ylim(bottom=0)

# =========================================================
# Animation
# =========================================================
def update(_frame):
    if not plot_buf_x:
        return [lp for g in all_lines for lp in g] + [err_line]

    x = list(plot_buf_x)
    n = len(x)

    for g_idx, g_lines in enumerate(all_lines):
        for c_idx, lp in enumerate(g_lines):
            y  = list(plot_bufs[g_idx][c_idx])
            mn = min(n, len(y))
            lp.set_data(x[:mn], y[:mn])
        axes[g_idx].relim()
        axes[g_idx].autoscale_view()

    e  = list(err_buf)
    mn = min(n, len(e))
    err_line.set_data(x[:mn], e[:mn])
    ax_err.relim()
    ax_err.autoscale_view()
    ax_err.set_ylim(bottom=0)

    return [lp for g in all_lines for lp in g] + [err_line]


ani = animation.FuncAnimation(
    fig, update,
    interval=PLOT_UPDATE_MS,
    blit=False,
    cache_frame_data=False,
)

# =========================================================
# Snapshot  (s 키)
# =========================================================
snapshot_counter = 0

def on_key(event):
    global snapshot_counter
    if event.key != SNAPSHOT_KEY:
        return
    snapshot_counter += 1
    snap_path = fig_dir / f"snapshot_{session_ts}_{snapshot_counter:03d}.png"
    fig.savefig(snap_path, dpi=150, bbox_inches="tight", facecolor="#1a1a1a")
    print(f"[Snapshot] Saved → {snap_path}")

fig.canvas.mpl_connect("key_press_event", on_key)

# =========================================================
# Main
# =========================================================
try:
    plt.show()
except KeyboardInterrupt:
    pass
finally:
    running = False
    writer_thread.join()
    ser.close()
    print("[Logger] Stopped.")
