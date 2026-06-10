# jitter_monitor.py
import csv
import struct
from datetime import datetime
from pathlib import Path

import matplotlib.animation as animation
import matplotlib.gridspec as gridspec
import matplotlib.pyplot as plt
import numpy as np
import serial
from collections import deque

from scripts.deframing.frame_parser import FrameParser

# ── Config ────────────────────────────────────────────────
MODE_DATA    = 'MOCK'   # 'REAL' | 'MOCK'
SERIAL_PORT  = 'COM3'
SERIAL_BAUD  = 115200
TARGET_MS    = 10.0
BUF_SIZE     = 500      # 시계열 그래프 표시 윈도우
HIST_BINS    = 50
UPDATE_MS    = 200
SNAPSHOT_KEY = 's'

# ── Export 경로 ───────────────────────────────────────────
EXPORT_ROOT  = Path(__file__).parent.parent / 'export'
_ts          = datetime.now().strftime('%Y_%m_%d_%H%M%S') + '_' + Path(__file__).stem
_session_dir = EXPORT_ROOT / _ts
_data_dir    = _session_dir / 'data'
_fig_dir     = _session_dir / 'fig'
for d in (_data_dir, _fig_dir):
    d.mkdir(parents=True, exist_ok=True)

_jitter_csv  = _data_dir / f'jitter_{_ts}.csv'
_stats_txt   = _data_dir / f'stats_{_ts}.txt'

print(f'[Jitter] Mode    : {MODE_DATA}')
print(f'[Jitter] CSV     : {_jitter_csv}')
print(f'[Jitter] Snapshots -> {_fig_dir}')
print('[Jitter] Press "s" to snapshot, close window to save stats.\n')

# ── CSV 파일 열기 ─────────────────────────────────────────
_csv_file   = open(_jitter_csv, 'w', newline='')
_csv_writer = csv.writer(_csv_file)
_csv_writer.writerow(['sample_idx', 'time_us', 'interval_ms'])

# ── 버퍼 ──────────────────────────────────────────────────
intervals    = deque(maxlen=BUF_SIZE)   # 화면 표시용 (최근 N개)
all_intervals: list[float] = []         # 전체 누적 (저장용)
prev_time_us = None
drop_count   = 0
crc_errors   = 0
total_count  = 0
_snapshot_n  = 0

# ── 시리얼 ────────────────────────────────────────────────
if MODE_DATA == 'MOCK':
    from scripts.mock.mock_serial import MockSerialBinary
    ser = MockSerialBinary()
else:
    ser = serial.Serial(SERIAL_PORT, SERIAL_BAUD, timeout=1)
parser = FrameParser()

def poll_serial():
    global prev_time_us, drop_count, crc_errors, total_count
    raw = ser.read(ser.in_waiting or 1)
    if not raw:
        return
    for pkt in parser.feed(raw):
        total_count += 1
        t_us = pkt['time_us']
        if prev_time_us is not None:
            dt_ms = (t_us - prev_time_us) / 1000.0
            intervals.append(dt_ms)
            all_intervals.append(dt_ms)
            _csv_writer.writerow([total_count - 1, t_us, f'{dt_ms:.6f}'])
            _csv_file.flush()
        prev_time_us = t_us
    drop_count = parser.drop_count
    crc_errors = parser.crc_errors


def _save_stats():
    if not all_intervals:
        return
    iv = np.array(all_intervals)
    lines = [
        f'Session  : {_ts}',
        f'Mode     : {MODE_DATA}',
        f'',
        f'Samples  : {total_count}',
        f'Drops    : {drop_count}',
        f'CRC Err  : {crc_errors}',
        f'',
        f'mean     : {np.mean(iv):.4f} ms',
        f'std      : {np.std(iv):.4f} ms',
        f'min      : {np.min(iv):.4f} ms',
        f'max      : {np.max(iv):.4f} ms',
        f'p95      : {np.percentile(iv, 95):.4f} ms',
        f'p99      : {np.percentile(iv, 99):.4f} ms',
    ]
    _stats_txt.write_text('\n'.join(lines))
    print(f'\n[Jitter] Stats saved -> {_stats_txt}')

def _on_close(_):
    _save_stats()
    _csv_file.close()
    ser.close()

# --- Figure --------------------------------------------------------
fig = plt.figure(figsize=(12, 8), facecolor='#ffffff')
gs  = gridspec.GridSpec(2, 2, figure=fig,
                        hspace=0.4, wspace=0.3,
                        left=0.08, right=0.97,
                        top=0.92,  bottom=0.08)

ax_line = fig.add_subplot(gs[0, :])
ax_hist = fig.add_subplot(gs[1, 0])
ax_stat = fig.add_subplot(gs[1, 1])

for ax in [ax_line, ax_hist]:
    ax.set_facecolor('#ffffff')
    ax.tick_params(colors='black', labelsize=8)
    ax.spines[['bottom', 'top', 'left', 'right']].set_color('#555')
    ax.grid(True, color='#dddddd', linewidth=0.5)

ax_stat.set_facecolor('#f5f5f5')
ax_stat.axis('off')

fig.text(0.5, 0.97, 'Jitter Monitor',
         color='black', fontsize=13, fontweight='bold',
         ha='center', va='top')

(line_interval,) = ax_line.plot(
    [], [], color='#378ADD', linewidth=1.0, label='Interval')
ax_line.axhline(TARGET_MS, color='#E24B4A', linewidth=1.0,
                linestyle='--', label=f'Target {TARGET_MS} ms')
ax_line.fill_between([], [], TARGET_MS, alpha=0.15, color='#378ADD')
ax_line.set_ylabel('Sample Interval [ms]', color='black', fontsize=9)
ax_line.set_xlabel('Sample Number', color='black', fontsize=9)
ax_line.legend(loc='upper right', fontsize=8,
               framealpha=0.3, labelcolor='black', facecolor='#eee')

ax_hist.set_xlabel('Sample Interval [ms]', color='black', fontsize=9)
ax_hist.set_ylabel('Count', color='black', fontsize=9)

stat_text = ax_stat.text(
    0.05, 0.95, '',
    transform=ax_stat.transAxes,
    color='black', fontsize=10,
    fontfamily='monospace',
    va='top', ha='left',
    linespacing=2.0,
)


# ── Update ────────────────────────────────────────────────
def update(_frame):
    poll_serial()
    if len(intervals) < 2:
        return

    iv = np.array(intervals)
    x  = np.arange(len(iv))

    line_interval.set_data(x, iv)
    for coll in ax_line.collections:
        coll.remove()
    ax_line.fill_between(x, iv, TARGET_MS,
                         where=(iv > TARGET_MS), alpha=0.2, color='#E24B4A')
    ax_line.fill_between(x, iv, TARGET_MS,
                         where=(iv < TARGET_MS), alpha=0.2, color='#44AA66')
    ax_line.relim()
    ax_line.autoscale_view()
    ax_line.set_ylim(max(0, TARGET_MS - 8), TARGET_MS + 8)

    ax_hist.cla()
    ax_hist.set_facecolor('#ffffff')
    ax_hist.tick_params(colors='black', labelsize=8)
    ax_hist.spines[['bottom', 'top', 'left', 'right']].set_color('#555')
    ax_hist.grid(True, color='#dddddd', linewidth=0.5)
    ax_hist.hist(iv, bins=HIST_BINS, color='#378ADD', alpha=0.8, edgecolor='none')
    ax_hist.axvline(TARGET_MS, color='#E24B4A', linewidth=1.2, linestyle='--')
    ax_hist.set_xlabel('Sample Interval [ms]', color='black', fontsize=9)
    ax_hist.set_ylabel('Count', color='black', fontsize=9)

    mean = np.mean(iv)
    std  = np.std(iv)
    mn   = np.min(iv)
    mx   = np.max(iv)
    p95  = np.percentile(iv, 95)
    p99  = np.percentile(iv, 99)

    stat_text.set_text(
        f'Sample Count  {total_count:>8d}\n'
        f'Drop Count    {drop_count:>8d}\n'
        f'CRC Errors    {crc_errors:>8d}\n'
        f'\n'
        f'mean          {mean:>8.3f} ms\n'
        f'std           {std:>8.3f} ms\n'
        f'min           {mn:>8.3f} ms\n'
        f'max           {mx:>8.3f} ms\n'
        f'p95           {p95:>8.3f} ms\n'
        f'p99           {p99:>8.3f} ms\n'
    )


# ── Snapshot ('s' 키) ─────────────────────────────────────
def on_key(event):
    global _snapshot_n
    if event.key != SNAPSHOT_KEY:
        return
    _snapshot_n += 1
    path = _fig_dir / f'snapshot_{_ts}_{_snapshot_n:03d}.png'
    fig.savefig(path, dpi=150, bbox_inches='tight', facecolor='#ffffff')
    print(f'[Jitter] Snapshot -> {path}')


fig.canvas.mpl_connect('key_press_event', on_key)
fig.canvas.mpl_connect('close_event', _on_close)

ani = animation.FuncAnimation(
    fig, update,
    interval=UPDATE_MS,
    blit=False,
    cache_frame_data=False,
)

plt.show()
