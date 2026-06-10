# serial_monitor.py
# Usage: python serial_monitor.py
#
# MODE = 'monitor'             → 창1: Euler Viz + State Estimation,  창2: IMU + Jitter
#      = 'euler_visualization' → Euler 3D 시각화 창 단독
#      = 'state_estimation'    → Roll/Pitch/Yaw 필터 비교 창 단독
#      = 'imu_monitor'         → IMU raw 창 단독
#      = 'jitter_monitor'      → Jitter 분석 창 단독
#
# MODE_DATA = 'MOCK'        → MockSerial (CSV 텍스트, readline)
#           = 'MOCK_BINARY' → MockSerialBinary (바이너리 프레임, read)
#           = 'REAL'        → 실제 시리얼 포트 (바이너리 프레임)
#
# 저장: export/{ts}/
#   data/imu_{ts}.csv     - seq·time_us·imu·filter·drop_flag
#   data/jitter_{ts}.csv  - sample_idx·time_us·interval_ms
#   data/stats_{ts}.txt   - 종료 시 요약 통계
#   fig/                  - 's' 키 스냅샷

import csv
import threading
from collections import deque
from datetime import datetime
from pathlib import Path
import sys

try:
    import matplotlib
    matplotlib.use('Qt5Agg')
except Exception:
    pass

import matplotlib.animation as animation
import matplotlib.pyplot as plt
import numpy as np
import serial

ROOT = Path(__file__).resolve().parent.parent
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from scripts.deframing.frame_parser import FrameParser

# -----------------------------------------------------------------------------
# Config
# -----------------------------------------------------------------------------
MODE        = 'monitor'      # 'monitor' | 'euler_visualization' | 'state_estimation' | 'imu_monitor' | 'jitter_monitor'
MODE_DATA   = 'MOCK_BINARY'  # 'REAL' | 'MOCK' | 'MOCK_BINARY'
SERIAL_PORT = 'COM6'
SERIAL_BAUD = 115200
PLOT_N      = 500            # 시계열 표시 샘플 수
UPDATE_MS   = 200            # 애니메이션 갱신 주기 [ms]

# -----------------------------------------------------------------------------
# Export
# -----------------------------------------------------------------------------
EXPORT_ROOT = Path(__file__).parent.parent / 'export'
_ts         = datetime.now().strftime('%Y_%m_%d_%H%M%S')
_data_dir   = EXPORT_ROOT / _ts / 'data'
_fig_dir    = EXPORT_ROOT / _ts / 'fig'
_log_dir    = EXPORT_ROOT / _ts / 'log'
_report_dir = EXPORT_ROOT / _ts / 'report'
for _d in (_data_dir, _fig_dir, _log_dir, _report_dir):
    _d.mkdir(parents=True, exist_ok=True)

# -----------------------------------------------------------------------------
# 시리얼 / 파서
# -----------------------------------------------------------------------------
if MODE_DATA == 'MOCK':
    from scripts.mock.mock_serial import MockSerial
    _ser = MockSerial()
elif MODE_DATA == 'MOCK_BINARY':
    from scripts.mock.mock_serial import MockSerialBinary
    _ser = MockSerialBinary()
else:
    _ser = serial.Serial(SERIAL_PORT, SERIAL_BAUD, timeout=1)

_parser = FrameParser()

print(f'[Monitor] MODE     : {MODE}  |  DATA: {MODE_DATA}')
print(f'[Monitor] Export   : {EXPORT_ROOT / _ts}')
print('[Monitor] Snapshot : press "s" on any window\n')

# -----------------------------------------------------------------------------
# SharedState  (deque CPython GIL로 append/read 원자적)
# -----------------------------------------------------------------------------
class _Shared:
    def __init__(self):
        self.total_count = 0
        self.drop_count  = 0
        self.crc_errors  = 0
        self.prev_t_us   = None

        # Jitter
        self.intervals: deque       = deque(maxlen=PLOT_N)
        self.all_ivals: list[float] = []
        self.all_t_us:  list[int]   = []

        # IMU raw
        self.ax_buf: deque = deque(maxlen=PLOT_N)
        self.ay_buf: deque = deque(maxlen=PLOT_N)
        self.az_buf: deque = deque(maxlen=PLOT_N)
        self.gx_buf: deque = deque(maxlen=PLOT_N)
        self.gy_buf: deque = deque(maxlen=PLOT_N)
        self.gz_buf: deque = deque(maxlen=PLOT_N)
        self.mx_buf: deque = deque(maxlen=PLOT_N)
        self.my_buf: deque = deque(maxlen=PLOT_N)
        self.mz_buf: deque = deque(maxlen=PLOT_N)

        # 필터 출력값 (모두 시리얼에서 수신)
        self.roll_gyro_buf:      deque = deque(maxlen=PLOT_N)
        self.pitch_gyro_buf:     deque = deque(maxlen=PLOT_N)
        self.yaw_gyro_buf:       deque = deque(maxlen=PLOT_N)
        self.roll_cf_buf:        deque = deque(maxlen=PLOT_N)
        self.pitch_cf_buf:       deque = deque(maxlen=PLOT_N)
        self.yaw_cf_buf:         deque = deque(maxlen=PLOT_N)
        self.roll_ekf_buf:       deque = deque(maxlen=PLOT_N)
        self.pitch_ekf_buf:      deque = deque(maxlen=PLOT_N)
        self.yaw_ekf_buf:        deque = deque(maxlen=PLOT_N)
        self.roll_madgwick_buf:  deque = deque(maxlen=PLOT_N)
        self.pitch_madgwick_buf: deque = deque(maxlen=PLOT_N)
        self.yaw_madgwick_buf:   deque = deque(maxlen=PLOT_N)
        self.roll_mahony_buf:    deque = deque(maxlen=PLOT_N)
        self.pitch_mahony_buf:   deque = deque(maxlen=PLOT_N)
        self.yaw_mahony_buf:     deque = deque(maxlen=PLOT_N)

sh = _Shared()

# -----------------------------------------------------------------------------
# CSV  (IO 스레드에서만 접근)
# -----------------------------------------------------------------------------
_imu_f    = open(_data_dir / f'imu_{_ts}.csv',    'w', newline='')
_jitter_f = open(_data_dir / f'jitter_{_ts}.csv', 'w', newline='')
_imu_w    = csv.writer(_imu_f)
_jitter_w = csv.writer(_jitter_f)
_imu_w.writerow(['seq', 'time_us',
                 'ax', 'ay', 'az', 'gx', 'gy', 'gz', 'mx', 'my', 'mz',
                 'roll_gyro',     'pitch_gyro',     'yaw_gyro',
                 'roll_cf',       'pitch_cf',       'yaw_cf',
                 'roll_ekf',      'pitch_ekf',      'yaw_ekf',
                 'roll_madgwick', 'pitch_madgwick', 'yaw_madgwick',
                 'roll_mahony',   'pitch_mahony',   'yaw_mahony',
                 'drop_flag'])
_jitter_w.writerow(['sample_idx', 'time_us', 'interval_ms'])

# -----------------------------------------------------------------------------
# IO Thread
# -----------------------------------------------------------------------------
_running   = threading.Event()
_running.set()
_prev_seq: int | None = None

def _process_pkt(pkt: dict) -> None:
    global _prev_seq

    sh.total_count += 1
    t   = pkt['time_us']
    seq = pkt['seq']

    drop_flag = 0
    if _prev_seq is not None and ((seq - _prev_seq) & 0xFFFF) != 1:
        drop_flag = 1
    _prev_seq = seq

    # IMU 버퍼
    ax, ay, az = pkt['ax'], pkt['ay'], pkt['az']
    gx, gy, gz = pkt['gx'], pkt['gy'], pkt['gz']
    mx, my, mz = pkt['mx'], pkt['my'], pkt['mz']
    sh.ax_buf.append(ax); sh.ay_buf.append(ay); sh.az_buf.append(az)
    sh.gx_buf.append(gx); sh.gy_buf.append(gy); sh.gz_buf.append(gz)
    sh.mx_buf.append(mx); sh.my_buf.append(my); sh.mz_buf.append(mz)

    # 필터 버퍼 (시리얼 수신값)
    sh.roll_gyro_buf.append(pkt['roll_gyro'])
    sh.pitch_gyro_buf.append(pkt['pitch_gyro'])
    sh.yaw_gyro_buf.append(pkt['yaw_gyro'])
    sh.roll_cf_buf.append(pkt['roll_cf'])
    sh.pitch_cf_buf.append(pkt['pitch_cf'])
    sh.yaw_cf_buf.append(pkt['yaw_cf'])
    sh.roll_ekf_buf.append(pkt['roll_ekf'])
    sh.pitch_ekf_buf.append(pkt['pitch_ekf'])
    sh.yaw_ekf_buf.append(pkt['yaw_ekf'])
    sh.roll_madgwick_buf.append(pkt['roll_madgwick'])
    sh.pitch_madgwick_buf.append(pkt['pitch_madgwick'])
    sh.yaw_madgwick_buf.append(pkt['yaw_madgwick'])
    sh.roll_mahony_buf.append(pkt['roll_mahony'])
    sh.pitch_mahony_buf.append(pkt['pitch_mahony'])
    sh.yaw_mahony_buf.append(pkt['yaw_mahony'])

    # IMU CSV
    _imu_w.writerow([
        seq, t, ax, ay, az, gx, gy, gz, mx, my, mz,
        pkt['roll_gyro'],     pkt['pitch_gyro'],     pkt['yaw_gyro'],
        pkt['roll_cf'],       pkt['pitch_cf'],       pkt['yaw_cf'],
        pkt['roll_ekf'],      pkt['pitch_ekf'],      pkt['yaw_ekf'],
        pkt['roll_madgwick'], pkt['pitch_madgwick'], pkt['yaw_madgwick'],
        pkt['roll_mahony'],   pkt['pitch_mahony'],   pkt['yaw_mahony'],
        drop_flag,
    ])

    # Jitter
    if sh.prev_t_us is not None:
        dt_us = t - sh.prev_t_us
        dt_ms = dt_us / 1000.0
        sh.intervals.append(dt_ms)
        sh.all_ivals.append(dt_ms)
        sh.all_t_us.append(t)
        _jitter_w.writerow([sh.total_count - 1, t, f'{dt_ms:.6f}'])
    sh.prev_t_us = t


_csv_seq = 0

def _parse_csv_line(line: str) -> dict | None:
    global _csv_seq
    try:
        parts = [p.strip() for p in line.split(',') if p.strip()]
        if len(parts) < 24:
            return None
        i = 0
        t_ms       = float(parts[i]); i += 1
        ax, ay, az = float(parts[i]), float(parts[i+1]), float(parts[i+2]); i += 3
        gx, gy, gz = float(parts[i]), float(parts[i+1]), float(parts[i+2]); i += 3
        mx, my, mz = float(parts[i]), float(parts[i+1]), float(parts[i+2]); i += 3
        rg, pg, yg = float(parts[i]), float(parts[i+1]), float(parts[i+2]); i += 3
        rc, pc, yc = float(parts[i]), float(parts[i+1]), float(parts[i+2]); i += 3
        re, pe, ye = float(parts[i]), float(parts[i+1]), float(parts[i+2]); i += 3
        rm, pm, ym = float(parts[i]), float(parts[i+1]), float(parts[i+2]); i += 3
        rh, ph, yh = float(parts[i]), float(parts[i+1]), float(parts[i+2])
        _csv_seq   = (_csv_seq + 1) & 0xFFFF
        return {
            'seq': _csv_seq, 'time_us': int(t_ms * 1000),
            'ax': ax, 'ay': ay, 'az': az,
            'gx': gx, 'gy': gy, 'gz': gz,
            'mx': mx, 'my': my, 'mz': mz,
            'roll_gyro':     rg, 'pitch_gyro':     pg, 'yaw_gyro':     yg,
            'roll_cf':       rc, 'pitch_cf':       pc, 'yaw_cf':       yc,
            'roll_ekf':      re, 'pitch_ekf':      pe, 'yaw_ekf':      ye,
            'roll_madgwick': rm, 'pitch_madgwick': pm, 'yaw_madgwick': ym,
            'roll_mahony':   rh, 'pitch_mahony':   ph, 'yaw_mahony':   yh,
        }
    except (ValueError, IndexError):
        return None

def _io_loop() -> None:
    while _running.is_set():
        try:
            if MODE_DATA == 'MOCK':
                raw = _ser.readline()
                if raw:
                    line = raw.decode('utf-8', errors='ignore').strip()
                    if line:
                        pkt = _parse_csv_line(line)
                        if pkt:
                            _process_pkt(pkt)
                _imu_f.flush()
                _jitter_f.flush()
            else:  # MOCK_BINARY or REAL
                raw = _ser.read(_ser.in_waiting or 1)
                if raw:
                    for pkt in _parser.feed(raw):
                        _process_pkt(pkt)
                    sh.drop_count = _parser.drop_count
                    sh.crc_errors = _parser.crc_errors
                    _imu_f.flush()
                    _jitter_f.flush()
        except Exception as exc:
            if _running.is_set():
                print(f'[IO] {exc}')

_io_thread = threading.Thread(target=_io_loop, daemon=True, name='io_thread')
_io_thread.start()

# -----------------------------------------------------------------------------
# Panel 생성
# monitor       → 창1: MonitorAhrsPanel (3D + 필터 비교),  창2: MonitorImuPanel (IMU + Jitter)
# 개별 MODE     → 각 standalone panel 1개
# -----------------------------------------------------------------------------
_panels: list = []

if MODE == 'monitor':
    from scripts.panel.panel_monitor_ahrs import MonitorAhrsPanel
    from scripts.panel.panel_monitor_imu  import MonitorImuPanel
    _panels.append(MonitorAhrsPanel(sh, _fig_dir, _ts))
    _panels.append(MonitorImuPanel(sh, _fig_dir, _ts))

elif MODE == 'euler_visualization':
    from scripts.panel.panel_euler_visualization import EulerVisualizationPanel
    _panels.append(EulerVisualizationPanel(sh, _fig_dir, _ts))

elif MODE == 'state_estimation':
    from scripts.panel.panel_state_estimation import StateEstimationPanel
    _panels.append(StateEstimationPanel(sh, _fig_dir, _ts))

elif MODE == 'imu_monitor':
    from scripts.panel.panel_imu_monitor import ImuMonitorPanel
    _panels.append(ImuMonitorPanel(sh, _fig_dir, _ts))

elif MODE == 'jitter_monitor':
    from scripts.panel.panel_jitter_monitor import JitterMonitorPanel
    _panels.append(JitterMonitorPanel(sh, _fig_dir, _ts))

# -----------------------------------------------------------------------------
# 종료 처리
# -----------------------------------------------------------------------------
_closed = False

def _on_close(_):
    global _closed
    if _closed:
        return
    _closed = True
    _running.clear()
    _io_thread.join(timeout=2.0)
    _save_stats()
    _imu_f.close()
    _jitter_f.close()
    _ser.close()

def _save_stats():
    if not sh.all_ivals:
        return
    iv   = np.array(sh.all_ivals)
    path = _data_dir / f'stats_{_ts}.txt'
    path.write_text('\n'.join([
        f'Session   : {_ts}',
        f'Mode      : {MODE} / {MODE_DATA}',
        '',
        f'Samples   : {sh.total_count}',
        f'Drops     : {sh.drop_count}',
        f'CRC Err   : {sh.crc_errors}',
        '',
        f'mean      : {np.mean(iv):.4f} ms',
        f'std       : {np.std(iv):.4f} ms',
        f'min       : {np.min(iv):.4f} ms',
        f'max       : {np.max(iv):.4f} ms',
        f'p95       : {np.percentile(iv, 95):.4f} ms',
        f'p99       : {np.percentile(iv, 99):.4f} ms',
    ]))
    print(f'[Monitor] Stats   -> {path}')
    print(f'[Monitor] IMU CSV -> {_data_dir / f"imu_{_ts}.csv"}')
    print(f'[Monitor] Jitter  -> {_data_dir / f"jitter_{_ts}.csv"}')

# -----------------------------------------------------------------------------
# 애니메이션  (matplotlib 패널만)
# -----------------------------------------------------------------------------
_anis: list = []

for _panel in _panels:
    if getattr(_panel, 'is_self_animated', False):
        if hasattr(_panel, 'register_close_callback'):
            _panel.register_close_callback(_on_close)
        continue

    def _make_update(p):
        def _upd(_): p.redraw()
        return _upd
    _interval = getattr(_panel, 'INTERVAL_MS', UPDATE_MS)
    _ani = animation.FuncAnimation(
        _panel.fig, _make_update(_panel),
        interval=_interval, blit=False, cache_frame_data=False,
    )
    _panel.fig.canvas.mpl_connect('close_event', _on_close)
    _anis.append(_ani)

# -----------------------------------------------------------------------------
# 이벤트 루프
# -----------------------------------------------------------------------------
_has_mpl = any(not getattr(p, 'is_self_animated', False) for p in _panels)
_has_qt  = any(getattr(p, 'is_self_animated', False)     for p in _panels)

if _has_mpl:
    plt.show()
elif _has_qt:
    import pyqtgraph as pg
    pg.mkQApp().exec_()