# panel_monitor_ahrs.py  —  monitor mode 창1: 3D Euler (좌) + State Estimation (우)
import time
import numpy as np
import pyqtgraph as pg
import pyqtgraph.opengl as gl
from PyQt5.QtWidgets import (QMainWindow, QWidget,
                              QHBoxLayout, QVBoxLayout, QLabel, QSplitter)
from PyQt5.QtCore import QTimer, Qt
from PyQt5.QtGui import QFont

try:
    from pyqtgraph.opengl import GLTextItem as _GLTextItem
    _HAS_TEXT = True
except ImportError:
    _HAS_TEXT = False

_AXIS = [
    {'name': 'X', 'gl': (0.89, 0.29, 0.29), 'pg': '#E24B4A'},
    {'name': 'Y', 'gl': (0.27, 0.67, 0.40), 'pg': '#44AA66'},
    {'name': 'Z', 'gl': (0.22, 0.54, 0.87), 'pg': '#378ADD'},
]

_FILTER_COLORS = {
    'gyro':     '#9370DB',
    'cf':       '#E24B4A',
    'ekf':      '#EF9F27',
    'madgwick': '#44AA66',
    'mahony':   '#378ADD',
}
_FILTERS = list(_FILTER_COLORS.keys())


def _rot_zyx(roll_deg, pitch_deg, yaw_deg):
    r, p, y = np.radians([roll_deg, pitch_deg, yaw_deg])
    Rx = np.array([[1, 0,          0         ],
                   [0, np.cos(r), -np.sin(r) ],
                   [0, np.sin(r),  np.cos(r) ]])
    Ry = np.array([[ np.cos(p), 0, np.sin(p)],
                   [0,          1, 0         ],
                   [-np.sin(p), 0, np.cos(p)]])
    Rz = np.array([[np.cos(y), -np.sin(y), 0],
                   [np.sin(y),  np.cos(y), 0],
                   [0,          0,          1]])
    return Rz @ Ry @ Rx


class _Window(QMainWindow):
    def __init__(self, panel):
        super().__init__()
        self._panel    = panel
        self._close_cb = None

    def keyPressEvent(self, event):
        if event.key() == Qt.Key_S:
            self._panel._snapshot()
        else:
            super().keyPressEvent(event)

    def closeEvent(self, event):
        if self._close_cb:
            self._close_cb(None)
        super().closeEvent(event)


class MonitorAhrsPanel:
    INTERVAL_MS      = 33
    is_self_animated = True
    fig              = None   # matplotlib figure 없음 (호환용)

    def __init__(self, shared, fig_dir, ts):
        self._sh      = shared
        self._fig_dir = fig_dir
        self._ts      = ts
        self._snap_n  = 0
        self._last_t  = None
        self._fps     = 0.0
        pg.mkQApp()
        self._setup()

    # ── 창 레이아웃 ──────────────────────────────────────────────────────────
    def _setup(self):
        win = _Window(self)
        win.setWindowTitle('Monitor — Euler Visualization + State Estimation')
        win.resize(1400, 760)
        win.setStyleSheet('background-color: white;')

        splitter = QSplitter(Qt.Horizontal)
        splitter.setStyleSheet(
            'QSplitter { background-color: white; }'
            'QSplitter::handle { background-color: #dddddd; }'
        )
        win.setCentralWidget(splitter)

        # ── 좌: 3D ───────────────────────────────────────────────────────────
        left_w = QWidget()
        left_l = QVBoxLayout(left_w)
        left_l.setContentsMargins(4, 4, 4, 4)
        left_l.setSpacing(4)

        lbl_l = QLabel('Euler Visualization')
        lbl_l.setAlignment(Qt.AlignCenter)
        lbl_l.setFont(QFont('Arial', 12, QFont.Bold))
        left_l.addWidget(lbl_l)

        self._glv = gl.GLViewWidget()
        self._glv.setBackgroundColor('w')
        self._glv.setCameraPosition(distance=4.5, elevation=25, azimuth=-45)
        self._glv.setMinimumHeight(400)
        left_l.addWidget(self._glv, 1)

        _ov = 'color: black; background-color: rgba(255,255,255,200); padding: 4px; border-radius: 3px;'
        self._rpy_lbl = QLabel('', self._glv)
        self._rpy_lbl.setFont(QFont('Courier New', 9))
        self._rpy_lbl.setStyleSheet(_ov)
        self._rpy_lbl.setAttribute(Qt.WA_TransparentForMouseEvents)
        self._rpy_lbl.move(8, 8)
        self._rpy_lbl.show()

        self._fps_lbl = QLabel('', self._glv)
        self._fps_lbl.setFont(QFont('Courier New', 8))
        self._fps_lbl.setStyleSheet(
            'color: #666; background-color: rgba(255,255,255,200); padding: 3px; border-radius: 3px;'
        )
        self._fps_lbl.setAttribute(Qt.WA_TransparentForMouseEvents)
        self._fps_lbl.show()

        splitter.addWidget(left_w)

        # ── 우: State Estimation (Roll / Pitch / Yaw) ─────────────────────────
        right_w = QWidget()
        right_l = QVBoxLayout(right_w)
        right_l.setContentsMargins(4, 4, 4, 4)
        right_l.setSpacing(3)

        lbl_r = QLabel('State Estimation')
        lbl_r.setAlignment(Qt.AlignCenter)
        lbl_r.setFont(QFont('Arial', 12, QFont.Bold))
        right_l.addWidget(lbl_r)

        def _pw(ylabel, show_x=False):
            w = pg.PlotWidget()
            w.setBackground('w')
            w.setLabel('left', ylabel, color='#333', size='8pt')
            w.getAxis('left').setTextPen('#333')
            w.showGrid(x=False, y=True, alpha=0.25)
            if show_x:
                w.setLabel('bottom', 'Sample', color='#333', size='8pt')
                w.getAxis('bottom').setTextPen('#333')
            else:
                w.getAxis('bottom').setStyle(showValues=False)
            return w

        pw_roll  = _pw('Roll [deg]')
        pw_pitch = _pw('Pitch [deg]')
        pw_yaw   = _pw('Yaw [deg]', show_x=True)

        self._plot_lines = {}
        for pw, angle in ((pw_roll, 'roll'), (pw_pitch, 'pitch'), (pw_yaw, 'yaw')):
            pw.addLegend(offset=(-8, 8), labelTextColor='#333')
            lines = {}
            for f in _FILTERS:
                lines[f] = pw.plot(pen=pg.mkPen(_FILTER_COLORS[f], width=1.5), name=f)
            self._plot_lines[angle] = lines

        for w in (pw_roll, pw_pitch, pw_yaw):
            right_l.addWidget(w, 1)

        splitter.addWidget(right_w)
        splitter.setSizes([560, 840])

        self._setup_3d()

        self._timer = QTimer()
        self._timer.setInterval(self.INTERVAL_MS)
        self._timer.timeout.connect(self.redraw)
        self._timer.start()

        self._win = win
        win.show()

    # ── 3D 장면 ──────────────────────────────────────────────────────────────
    def _setup_3d(self):
        gv = self._glv

        grid = gl.GLGridItem()
        grid.setSize(2.8, 2.8)
        grid.setSpacing(0.40, 0.40)
        grid.translate(0, 0, -1.4)
        grid.setColor((130, 130, 130, 200))
        gv.addItem(grid)

        _wf_off = np.array([-1.3, -1.3, -1.3], dtype=np.float32)
        self._add_static_frame(gv, _wf_off, length=0.28, lw=1.5, alpha=0.5)

        self._sf_lines: list = []
        self._sf_texts: list = []
        o = np.zeros(3, dtype=np.float32)
        for i, ax in enumerate(_AXIS):
            r, g, b = ax['gl']
            line = gl.GLLinePlotItem(
                pos=np.array([o, o + np.eye(3, dtype=np.float32)[i]]),
                color=(r, g, b, 1.0), width=3.0, antialias=True,
            )
            gv.addItem(line)
            self._sf_lines.append(line)
            if _HAS_TEXT:
                tip = np.eye(3, dtype=np.float32)[i] * 1.25
                txt = _GLTextItem()
                txt.setData(pos=tip, text=ax['name'], color=pg.mkColor(ax['pg']))
                gv.addItem(txt)
                self._sf_texts.append(txt)

    def _add_static_frame(self, gv, offset, length, lw, alpha):
        R = np.eye(3)
        for i, ax in enumerate(_AXIS):
            r, g, b = ax['gl']
            d   = R[:, i].astype(np.float32)
            end = (offset + d * length).astype(np.float32)
            line = gl.GLLinePlotItem(
                pos=np.array([offset, end]),
                color=(r, g, b, alpha), width=lw, antialias=True,
            )
            gv.addItem(line)
            if _HAS_TEXT:
                tip = (offset + d * length * 1.3).astype(np.float32)
                txt = _GLTextItem()
                txt.setData(pos=tip, text=ax['name'], color=pg.mkColor(ax['pg']))
                gv.addItem(txt)

    # ── 갱신 ─────────────────────────────────────────────────────────────────
    def redraw(self):
        now = time.perf_counter()
        if self._last_t and (now - self._last_t) > 0:
            self._fps = 0.85 * self._fps + 0.15 / (now - self._last_t)
        self._last_t = now

        sh = self._sh
        n  = len(sh.roll_cf_buf)
        if n < 2:
            return

        x = np.arange(n, dtype=np.float32)

        buf_map = {
            'roll':  {
                'gyro': sh.roll_gyro_buf,     'cf': sh.roll_cf_buf,
                'ekf':  sh.roll_ekf_buf,       'madgwick': sh.roll_madgwick_buf,
                'mahony': sh.roll_mahony_buf,
            },
            'pitch': {
                'gyro': sh.pitch_gyro_buf,    'cf': sh.pitch_cf_buf,
                'ekf':  sh.pitch_ekf_buf,      'madgwick': sh.pitch_madgwick_buf,
                'mahony': sh.pitch_mahony_buf,
            },
            'yaw': {
                'gyro': sh.yaw_gyro_buf,      'cf': sh.yaw_cf_buf,
                'ekf':  sh.yaw_ekf_buf,        'madgwick': sh.yaw_madgwick_buf,
                'mahony': sh.yaw_mahony_buf,
            },
        }
        for angle, fbufs in buf_map.items():
            for f, buf in fbufs.items():
                y = np.array(buf, dtype=np.float32)
                m = min(len(x), len(y))
                self._plot_lines[angle][f].setData(x[:m], y[:m])

        # 3D는 CF 필터 기준
        roll  = sh.roll_cf_buf[-1]  if sh.roll_cf_buf  else 0.0
        pitch = sh.pitch_cf_buf[-1] if sh.pitch_cf_buf else 0.0
        yaw   = sh.yaw_cf_buf[-1]   if sh.yaw_cf_buf   else 0.0
        self._update_3d(roll, pitch, yaw)

    def _update_3d(self, roll_deg, pitch_deg, yaw_deg):
        R = _rot_zyx(roll_deg, pitch_deg, yaw_deg)
        o = np.zeros(3, dtype=np.float32)
        for i in range(3):
            d   = R[:, i].astype(np.float32)
            self._sf_lines[i].setData(pos=np.array([o, d * 1.0]))
            if _HAS_TEXT and i < len(self._sf_texts):
                self._sf_texts[i].setData(pos=(d * 1.25).astype(np.float32))

        self._rpy_lbl.setText(
            f'Roll  {roll_deg:+8.2f}°\n'
            f'Pitch {pitch_deg:+8.2f}°\n'
            f'Yaw   {yaw_deg:+8.2f}°\n'
            f'(CF filter)'
        )
        self._rpy_lbl.adjustSize()

        fh = self._fps_lbl.sizeHint().height()
        self._fps_lbl.move(8, self._glv.height() - fh - 8)
        self._fps_lbl.setText(f'FPS  {self._fps:.1f}')
        self._fps_lbl.adjustSize()

    # ── 스냅샷 ────────────────────────────────────────────────────────────────
    def _snapshot(self):
        self._snap_n += 1
        path = self._fig_dir / f'ahrs_snap_{self._ts}_{self._snap_n:03d}.png'
        self._win.grab().save(str(path), 'PNG')
        print(f'[MonitorAHRS] Snapshot -> {path}')

    def register_close_callback(self, cb):
        def _cb(_):
            self._timer.stop()
            cb(None)
        self._win._close_cb = _cb
