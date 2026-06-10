"""
File Name: serial_logger.py
Author: Beomjun Chung
Updated: 2026-06-10

Description:
    센서 데이터를 실시간으로 시각화하는 스크립트

    Live 창 레이아웃:
    ┌──────────────────────┬──────────────────────┐
    │  3D Euler Angle Viz  │  Roll  (5개 필터)     │
    │  immediate value     │  Pitch (5개 필터)     │
    │  sensor frame,       │  Yaw   (5개 필터)     │
    │  world frame         │                      │
    └──────────────────────┴──────────────────────┘

    Snapshot 키: s
        - 현재 화면 snapshot PNG 저장

    MODE_DATA = 'MOCK'        → MockSerial (CSV 텍스트, readline)
              = 'MOCK_BINARY' → MockSerialBinary (바이너리 프레임, read)
              = 'REAL'        → 실제 시리얼 포트 (CSV 텍스트, readline)
"""

import serial
import csv
import threading
import queue
from collections import deque
from datetime import datetime
from pathlib import Path
import sys

import numpy as np
import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec
import matplotlib.animation as animation
from mpl_toolkits.mplot3d import Axes3D  # noqa: F401

ROOT = Path(__file__).resolve().parent.parent
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from scripts.deframing.frame_parser import FrameParser

# =========================================================
# Config
# =========================================================
MODE_DATA   = "MOCK_BINARY"   # "REAL" | "MOCK" | "MOCK_BINARY"

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
    "roll_gyro", "pitch_gyro", "yaw_gyro",
    "roll_cf",   "pitch_cf",   "yaw_cf",
    "roll_ekf",  "pitch_ekf",  "yaw_ekf",
    "roll_madgwick", "pitch_madgwick", "yaw_madgwick",
    "roll_mahony",   "pitch_mahony",   "yaw_mahony",
]
N_COLS     = len(HEADER)   # 25
PLOT_COL_X = 0             # t_ms

# 컬럼 인덱스 (0-based)
ROLL_GYRO_COL  = 10;  PITCH_GYRO_COL  = 11;  YAW_GYRO_COL  = 12
ROLL_CF_COL    = 13;  PITCH_CF_COL    = 14;  YAW_CF_COL    = 15
ROLL_EKF_COL   = 16;  PITCH_EKF_COL   = 17;  YAW_EKF_COL   = 18
ROLL_MAD_COL   = 19;  PITCH_MAD_COL   = 20;  YAW_MAD_COL   = 21
ROLL_MAH_COL   = 22;  PITCH_MAH_COL   = 23;  YAW_MAH_COL   = 24

ATTITUDE_GROUPS = [
    {
        "ylabel": "Roll [deg]",
        "channels": [
            {"col": ROLL_GYRO_COL, "label": "Gyro"},
            {"col": ROLL_CF_COL,   "label": "CF"},
            {"col": ROLL_EKF_COL,  "label": "EKF"},
            {"col": ROLL_MAD_COL,  "label": "Madgwick"},
            {"col": ROLL_MAH_COL,  "label": "Mahony"},
        ],
    },
    {
        "ylabel": "Pitch [deg]",
        "channels": [
            {"col": PITCH_GYRO_COL, "label": "Gyro"},
            {"col": PITCH_CF_COL,   "label": "CF"},
            {"col": PITCH_EKF_COL,  "label": "EKF"},
            {"col": PITCH_MAD_COL,  "label": "Madgwick"},
            {"col": PITCH_MAH_COL,  "label": "Mahony"},
        ],
    },
    {
        "ylabel": "Yaw [deg]",
        "channels": [
            {"col": YAW_GYRO_COL, "label": "Gyro"},
            {"col": YAW_CF_COL,   "label": "CF"},
            {"col": YAW_EKF_COL,  "label": "EKF"},
            {"col": YAW_MAD_COL,  "label": "Madgwick"},
            {"col": YAW_MAH_COL,  "label": "Mahony"},
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
log_dir     = session_dir / "log"
report_dir  = session_dir / "report"

for d in (data_dir, fig_dir, log_dir, report_dir):
    d.mkdir(parents=True, exist_ok=True)

csv_path = data_dir / f"imu_{session_ts}.csv"
print(f"[Logger] MODE_DATA : {MODE_DATA}")
print(f"[Logger] Saving    → {csv_path}")

# =========================================================
# Globals
# =========================================================
data_queue = queue.Queue(maxsize=QUEUE_SIZE)
plot_buf_x = deque(maxlen=PLOT_BUFFER_SIZE)
running    = True

att_bufs = [
    [deque(maxlen=PLOT_BUFFER_SIZE) for _ in grp["channels"]]
    for grp in ATTITUDE_GROUPS
]
cur_rpy = [0.0, 0.0, 0.0]

# =========================================================
# Serial
# =========================================================
if MODE_DATA == "REAL":
    ser    = serial.Serial(SERIAL_PORT, SERIAL_BAUD, timeout=1)
    _parser = None
elif MODE_DATA == "MOCK":
    from scripts.mock.mock_serial import MockSerial
    ser    = MockSerial()
    _parser = None
elif MODE_DATA == "MOCK_BINARY":
    from scripts.mock.mock_serial import MockSerialBinary
    ser    = MockSerialBinary()
    _parser = FrameParser()
else:
    raise ValueError(f"Unknown MODE_DATA: {MODE_DATA}")

# =========================================================
# Threads
# =========================================================
def _pkt_to_csv_line(pkt: dict) -> str:
    """바이너리 패킷 → CSV 텍스트 변환 (N_COLS = 25)"""
    t_ms = pkt['time_us'] / 1000.0
    return (
        f"{t_ms:.3f},"
        f"{pkt['ax']},{pkt['ay']},{pkt['az']},"
        f"{pkt['gx']},{pkt['gy']},{pkt['gz']},"
        f"{pkt['mx']},{pkt['my']},{pkt['mz']},"
        f"{pkt['roll_gyro']},{pkt['pitch_gyro']},{pkt['yaw_gyro']},"
        f"{pkt['roll_cf']},{pkt['pitch_cf']},{pkt['yaw_cf']},"
        f"{pkt['roll_ekf']},{pkt['pitch_ekf']},{pkt['yaw_ekf']},"
        f"{pkt['roll_madgwick']},{pkt['pitch_madgwick']},{pkt['yaw_madgwick']},"
        f"{pkt['roll_mahony']},{pkt['pitch_mahony']},{pkt['yaw_mahony']}"
    )

def read_serial():
    while running:
        try:
            if MODE_DATA == "MOCK_BINARY":
                raw = ser.read(ser.in_waiting or 1)
                if raw:
                    for pkt in _parser.feed(raw):
                        line = _pkt_to_csv_line(pkt)
                        if data_queue.full():
                            data_queue.get_nowait()
                        data_queue.put(line)
            else:
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
                cols = line.split(",")
                if len(cols) != N_COLS:
                    continue
                writer.writerow(cols)
                f.flush()
                try:
                    plot_buf_x.append(float(cols[PLOT_COL_X]))

                    for g_idx, grp in enumerate(ATTITUDE_GROUPS):
                        for c_idx, ch in enumerate(grp["channels"]):
                            att_bufs[g_idx][c_idx].append(float(cols[ch["col"]]))

                    # 3D용 현재값 (CF 기준)
                    cur_rpy[0] = float(cols[ROLL_CF_COL])
                    cur_rpy[1] = float(cols[PITCH_CF_COL])
                    cur_rpy[2] = float(cols[YAW_CF_COL])
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
# 3D 유틸
# =========================================================
AXIS_COLORS = {"X": "#FF4444", "Y": "#44CC44", "Z": "#4488FF"}

def rot_zyx(roll_deg, pitch_deg, yaw_deg):
    r, p, y = np.radians([roll_deg, pitch_deg, yaw_deg])
    Rx = np.array([[1, 0,          0         ],
                   [0, np.cos(r), -np.sin(r) ],
                   [0, np.sin(r),  np.cos(r) ]])
    Ry = np.array([[ np.cos(p), 0, np.sin(p)],
                   [0,           1, 0        ],
                   [-np.sin(p), 0, np.cos(p)]])
    Rz = np.array([[np.cos(y), -np.sin(y), 0],
                   [np.sin(y),  np.cos(y), 0],
                   [0,          0,          1]])
    return Rz @ Ry @ Rx

def _draw_frame(ax, origin, R, length, lw, fontsize, alpha=1.0):
    for i, (name, color) in enumerate(AXIS_COLORS.items()):
        d = R[:, i]
        ax.quiver(
            origin[0], origin[1], origin[2],
            d[0] * length, d[1] * length, d[2] * length,
            color=color, linewidth=lw,
            arrow_length_ratio=0.2, alpha=alpha,
        )
        tip = origin + d * length * 1.28
        ax.text(
            tip[0], tip[1], tip[2], name,
            color=color, fontsize=fontsize,
            fontweight="bold", ha="center", va="center",
            alpha=alpha,
        )

def draw_3d_scene(ax, roll_deg, pitch_deg, yaw_deg):
    ax.cla()

    ax.set_facecolor("black")
    lim = 1.6
    ax.set_xlim(-lim, lim)
    ax.set_ylim(-lim, lim)
    ax.set_zlim(-lim, lim)
    ax.set_box_aspect([1, 1, 1])
    ax.set_xticks([])
    ax.set_yticks([])
    ax.set_zticks([])
    ax.xaxis.pane.fill = False
    ax.yaxis.pane.fill = False
    ax.zaxis.pane.fill = False
    ax.xaxis.pane.set_edgecolor("none")
    ax.yaxis.pane.set_edgecolor("none")
    ax.zaxis.pane.set_edgecolor("none")
    ax.grid(False)

    for spine in ax.spines.values():
        spine.set_edgecolor("#555555")
        spine.set_linewidth(0.8)

    for v in np.linspace(-1.4, 1.4, 8):
        ax.plot([-1.4, 1.4], [v,    v   ], [-1.4, -1.4], color="#252525", linewidth=0.5)
        ax.plot([v,    v   ], [-1.4, 1.4], [-1.4, -1.4], color="#252525", linewidth=0.5)

    origin = np.array([0.0, 0.0, 0.0])
    R = rot_zyx(roll_deg, pitch_deg, yaw_deg)
    _draw_frame(ax, origin, R, length=1.0, lw=2.5, fontsize=11, alpha=1.0)

    w_origin = np.array([-1.40, -1.40, -1.50])
    _draw_frame(ax, w_origin, np.eye(3), length=0.28, lw=1.2, fontsize=7, alpha=0.65)

    ax.scatter([0], [0], [0], color="white", s=20, zorder=10)

    label = (
        f"Roll  {roll_deg:+8.3f}°\n"
        f"Pitch {pitch_deg:+8.3f}°\n"
        f"Yaw   {yaw_deg:+8.3f}°"
    )
    ax.text2D(
        0.02, 0.98, label,
        transform=ax.transAxes,
        color="white", fontsize=9,
        fontfamily="monospace",
        va="top", ha="left",
        linespacing=1.8,
    )

# =========================================================
# Figure 레이아웃
# =========================================================
fig = plt.figure(figsize=(16, 8), facecolor="#1a1a1a")

fig.text(0.23, 0.97, "Euler Visualizer",
         color="white", fontsize=12, fontweight="bold", ha="center", va="top")
fig.text(0.68, 0.97, "State Estimation Results",
         color="white", fontsize=12, fontweight="bold", ha="center", va="top")

outer = gridspec.GridSpec(
    1, 2,
    width_ratios=[1.1, 1.6],
    left=0.03, right=0.97,
    top=0.93,  bottom=0.05,
    wspace=0.10,
)

ax_main = fig.add_subplot(outer[0], projection="3d")

right_gs = gridspec.GridSpecFromSubplotSpec(3, 1, subplot_spec=outer[1], hspace=0.30)
att_axes = [fig.add_subplot(right_gs[i]) for i in range(3)]

att_axes[1].sharex(att_axes[0])
att_axes[2].sharex(att_axes[0])

# ---------------------------------------------------------
# Attitude 라인 초기화
# ---------------------------------------------------------
LINE_COLORS = ["#9370DB", "#E24B4A", "#EF9F27", "#44AA66", "#378ADD"]

att_lines = []
for ax, grp in zip(att_axes, ATTITUDE_GROUPS):
    g_lines = []
    for c_idx, ch in enumerate(grp["channels"]):
        (lp,) = ax.plot(
            [], [],
            color=LINE_COLORS[c_idx % len(LINE_COLORS)],
            linewidth=1.2, label=ch["label"],
        )
        g_lines.append(lp)
    ax.set_ylabel(grp["ylabel"], color="white", fontsize=8)
    ax.tick_params(colors="white", labelsize=7)
    ax.spines[["bottom", "top", "left", "right"]].set_color("#555")
    ax.set_facecolor("#111111")
    ax.grid(True, color="#2a2a2a", linewidth=0.5)
    ax.legend(loc="upper right", fontsize=7, framealpha=0.3, labelcolor="white", facecolor="#222")
    att_lines.append(g_lines)

att_axes[-1].set_xlabel("Time [ms]", color="white", fontsize=8)
for ax in att_axes[:-1]:
    ax.tick_params(labelbottom=False)

# =========================================================
# Animation
# =========================================================
def update(_):
    if plot_buf_x:
        x = list(plot_buf_x)
        n = len(x)

        for g_idx, g_lines in enumerate(att_lines):
            for c_idx, lp in enumerate(g_lines):
                y  = list(att_bufs[g_idx][c_idx])
                mn = min(n, len(y))
                lp.set_data(x[:mn], y[:mn])
            att_axes[g_idx].relim()
            att_axes[g_idx].autoscale_view()

    draw_3d_scene(ax_main, *cur_rpy)

    return [lp for g in att_lines for lp in g]

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