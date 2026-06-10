"""
File Name: sensor_visualizer.py
Author: Beomjun Chung
Updated: 2026-06-02

Description:
    센서 데이터를 실시간으로 시각화하는 스크립트

    Live 창 레이아웃:
    ┌──────────────────────┬──────────────────────┐
    │  3D Euler Angle Viz  │  Roll  (CF / KF)     │
    │  immediate value     │  Pitch (CF / KF)     │
    │  sensor frame,       │  Yaw   (CF / KF)     │
    │  world frame         │  Error Norm          │
    └──────────────────────┴──────────────────────┘

    Snapshot 키: s
        - 현재 화면 snapshot PNG 저장
"""

import serial
import csv
import threading
import queue
from collections import deque
from datetime import datetime
from pathlib import Path
import struct

import numpy as np
import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec
import matplotlib.animation as animation
from mpl_toolkits.mplot3d import Axes3D  # noqa: F401

from scripts.deframing.frame_parser import FrameParser

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
    "time_ms",
    "ax", "ay", "az",
    "gx", "gy", "gz",
    "mx", "my", "mz",
    "roll_cf", "pitch_cf", "yaw_cf",
    "roll_kf",  "pitch_kf",  "yaw_kf",
]
N_COLS     = len(HEADER)
PLOT_COL_X = 0

# roll/pitch/yaw col 인덱스
ROLL_CF_COL  = 10;  PITCH_CF_COL = 11;  YAW_CF_COL  = 12
ROLL_KF_COL  = 13;  PITCH_KF_COL = 14;  YAW_KF_COL  = 15

ATTITUDE_GROUPS = [
    {
        "ylabel": "Roll [deg]",
        "channels": [
            {"col": ROLL_CF_COL,  "label": "CF"},
            {"col": ROLL_KF_COL,  "label": "KF"},
        ],
    },
    {
        "ylabel": "Pitch [deg]",
        "channels": [
            {"col": PITCH_CF_COL, "label": "CF"},
            {"col": PITCH_KF_COL, "label": "KF"},
        ],
    },
    {
        "ylabel": "Yaw [deg]",
        "channels": [
            {"col": YAW_CF_COL,   "label": "CF"},
            {"col": YAW_KF_COL,   "label": "KF"},
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
print(f"[Logger] Saving → {csv_path}")

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
err_buf = deque(maxlen=PLOT_BUFFER_SIZE)   # error norm 버퍼
cur_rpy = [0.0, 0.0, 0.0]

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
                cols = line.split(",")
                if len(cols) != N_COLS:
                    continue
                writer.writerow(cols)
                f.flush()
                try:
                    plot_buf_x.append(float(cols[PLOT_COL_X]))

                    # attitude 버퍼
                    for g_idx, grp in enumerate(ATTITUDE_GROUPS):
                        for c_idx, ch in enumerate(grp["channels"]):
                            att_bufs[g_idx][c_idx].append(float(cols[ch["col"]]))

                    # error norm: ||[roll_cf-roll_kf, pitch_cf-pitch_kf, yaw_cf-yaw_kf]||
                    dr = float(cols[ROLL_CF_COL])  - float(cols[ROLL_KF_COL])
                    dp = float(cols[PITCH_CF_COL]) - float(cols[PITCH_KF_COL])
                    dy = float(cols[YAW_CF_COL])   - float(cols[YAW_KF_COL])
                    err_buf.append(np.sqrt(dr**2 + dp**2 + dy**2))

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

    # ── 스타일 ───────────────────────────────────────────
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

    # 테두리 (2D patch로 외곽 사각형)
    for spine in ax.spines.values():
        spine.set_edgecolor("#555555")
        spine.set_linewidth(0.8)

    # ── 바닥 그리드 ──────────────────────────────────────
    for v in np.linspace(-1.4, 1.4, 8):
        ax.plot([-1.4, 1.4], [v,    v   ], [-1.4, -1.4], color="#252525", linewidth=0.5)
        ax.plot([v,    v   ], [-1.4, 1.4], [-1.4, -1.4], color="#252525", linewidth=0.5)

    origin = np.array([0.0, 0.0, 0.0])

    # ── sensor frame (크게, 불투명) ───────────────────────
    R = rot_zyx(roll_deg, pitch_deg, yaw_deg)
    _draw_frame(ax, origin, R, length=1.0, lw=2.5, fontsize=11, alpha=1.0)

    # ── world frame (좌하단 offset, 작게, 반투명) ─────────
    # 3D 좌표계에서 좌하단 구석 근처 위치
    w_origin = np.array([-1.40, -1.40, -1.50])
    _draw_frame(ax, w_origin, np.eye(3), length=0.28, lw=1.2, fontsize=7, alpha=0.65)

    ax.scatter([0], [0], [0], color="white", s=20, zorder=10)

    # ── Roll/Pitch/Yaw/Error 고정 위치 텍스트 ──────────────
    cur_err = err_buf[-1] if err_buf else 0.0
    label = (
        f"Roll  {roll_deg:+8.3f}°\n"
        f"Pitch {pitch_deg:+8.3f}°\n"
        f"Yaw   {yaw_deg:+8.3f}°\n"
        f"Error {cur_err:+8.3f}°"
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

# 좌/우 제목을 figure text로 고정 배치
fig.text(0.23, 0.97, "Euler Visualizer", color="white", fontsize=12, fontweight="bold", ha="center", va="top")
fig.text(0.68, 0.97, "State Estimation Results", color="white", fontsize=12, fontweight="bold", ha="center", va="top")

outer = gridspec.GridSpec(
    1, 2,
    width_ratios=[1.1, 1.6],
    left=0.03, right=0.97,
    top=0.93,  bottom=0.05,   # 위아래 여백 최소화 → 3D 높이 확장
    wspace=0.10,
)

ax_main = fig.add_subplot(outer[0], projection="3d")

# 오른쪽: Roll / Pitch / Yaw / Error Norm — 4개 subplot
right_gs = gridspec.GridSpecFromSubplotSpec(4, 1, subplot_spec=outer[1], hspace=0.30)
att_axes = [fig.add_subplot(right_gs[i]) for i in range(3)]
ax_err   = fig.add_subplot(right_gs[3])

att_axes[1].sharex(att_axes[0])
att_axes[2].sharex(att_axes[0])
ax_err.sharex(att_axes[0])

# ---------------------------------------------------------
# 오른쪽 attitude 라인 초기화
# ---------------------------------------------------------
LINE_COLORS = ["#378ADD", "#E24B4A", "#44AA66", "#EF9F27"]
LINE_STYLES = ["-", "--", "-.", ":"]

att_lines = []
for ax, grp in zip(att_axes, ATTITUDE_GROUPS):
    g_lines = []
    for c_idx, ch in enumerate(grp["channels"]):
        (lp,) = ax.plot(
            [], [],
            color=LINE_COLORS[c_idx % len(LINE_COLORS)],
            linestyle=LINE_STYLES[c_idx % len(LINE_STYLES)],
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

# error norm 라인
(err_line,) = ax_err.plot([], [], color="#EF9F27", linewidth=1.2, label="‖CF−KF‖")
ax_err.set_ylabel("Error Norm\n[deg]", color="white", fontsize=8)
ax_err.set_xlabel("Time [ms]", color="white", fontsize=8)
ax_err.tick_params(colors="white", labelsize=7)
ax_err.spines[["bottom", "top", "left", "right"]].set_color("#555")
ax_err.set_facecolor("#111111")
ax_err.grid(True, color="#2a2a2a", linewidth=0.5)
ax_err.legend(loc="upper right", fontsize=7,
              framealpha=0.3, labelcolor="white", facecolor="#222")
# ax_err.set_ylim(bottom=0)

# x축 라벨: 마지막 subplot에만
for ax in att_axes:
    ax.tick_params(labelbottom=False)

# =========================================================
# Animation
# =========================================================
def update(_frame):
    # ── 오른쪽: attitude 시계열 ──────────────────────────
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

        # error norm
        e  = list(err_buf)
        mn = min(n, len(e))
        err_line.set_data(x[:mn], e[:mn])
        ax_err.relim()
        ax_err.autoscale_view()
        ax_err.set_ylim(bottom=0)

    # ── 왼쪽: 3D scene ───────────────────────────────────
    draw_3d_scene(ax_main, *cur_rpy)

    return [lp for g in att_lines for lp in g] + [err_line]

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