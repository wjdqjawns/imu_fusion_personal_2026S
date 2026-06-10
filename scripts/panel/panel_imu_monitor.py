# panel_imu.py
# ImuPanel: serial_logger.py가 생성한 SharedState를 받아 raw IMU 시각화.
# binary frame 기준 (accel / gyro / mag).
# 상태 추정값(roll/pitch/yaw)을 프레임에 추가하면 이 파일만 확장.
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec

_COLORS = ['#E24B4A', '#44AA66', '#378ADD']

class ImuMonitorPanel:
    def __init__(self, shared, fig_dir, ts):
        self._sh      = shared
        self._fig_dir = fig_dir
        self._ts      = ts
        self._snap_n  = 0
        self._setup()

    # --- configuration figure -----------------------------------------------
    def _setup(self):
        fig = plt.figure(figsize=(12, 9), facecolor='#ffffff')
        fig.canvas.manager.set_window_title('IMU Monitor')
        gs = gridspec.GridSpec(3, 1, figure=fig, hspace=0.5, left=0.08, right=0.97, top=0.92,  bottom=0.06)

        ax_acc = fig.add_subplot(gs[0])
        ax_gyr = fig.add_subplot(gs[1])
        ax_mag = fig.add_subplot(gs[2])

        for ax in (ax_acc, ax_gyr, ax_mag):
            ax.set_facecolor('#ffffff')
            ax.tick_params(colors='black', labelsize=8)
            ax.spines[['bottom', 'top', 'left', 'right']].set_color('#555')
            ax.grid(True, color='#dddddd', linewidth=0.5)

        fig.text(0.5, 0.97, 'IMU Monitor', color='black', fontsize=13, fontweight='bold', ha='center', va='top')

        def _make_lines(ax, labels, ylabel):
            lines = []
            for label, color in zip(labels, _COLORS):
                (ln,) = ax.plot([], [], color=color, lw=1, label=label)
                lines.append(ln)
            ax.set_ylabel(ylabel, color='black', fontsize=9)
            ax.legend(loc='upper right', fontsize=7, framealpha=0.3, labelcolor='black', facecolor='#eee', ncol=3)
            return lines

        self._ln_acc = _make_lines(ax_acc, ['ax', 'ay', 'az'], 'Accel [g]')
        self._ln_gyr = _make_lines(ax_gyr, ['gx', 'gy', 'gz'], 'Gyro [deg/s]')
        self._ln_mag = _make_lines(ax_mag, ['mx', 'my', 'mz'], 'Mag [uT]')

        ax_mag.set_xlabel('Sample Number', color='black', fontsize=9)

        self.fig   = fig
        self._axes = (ax_acc, ax_gyr, ax_mag)

        fig.canvas.mpl_connect('key_press_event', self._on_key)

    # --- graph update -----------------------------------------------------
    def redraw(self):
        sh = self._sh
        n = len(sh.ax_buf)
        if n < 2:
            return

        x = np.arange(n)
        groups = [
            (self._ln_acc, (sh.ax_buf, sh.ay_buf, sh.az_buf)),
            (self._ln_gyr, (sh.gx_buf, sh.gy_buf, sh.gz_buf)),
            (self._ln_mag, (sh.mx_buf, sh.my_buf, sh.mz_buf)),
        ]
        for lines, bufs in groups:
            for ln, buf in zip(lines, bufs):
                y = np.array(buf)
                m = min(n, len(y))
                ln.set_data(x[:m], y[:m])

        for ax in self._axes:
            ax.relim()
            ax.autoscale_view()

    # --- snapshot ('s' key each panel) ---------------------------------------
    def _on_key(self, event):
        if event.key != 's':
            return
        self._snap_n += 1
        path = self._fig_dir / f'imu_snap_{self._ts}_{self._snap_n:03d}.png'
        self.fig.savefig(path, dpi=150, bbox_inches='tight', facecolor='#ffffff')
        print(f'[IMU] Snapshot -> {path}')