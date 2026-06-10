# panel_state_estimation.py  —  standalone: Roll / Pitch / Yaw (5개 필터 비교)
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec

_FILTER_COLORS = {
    'gyro':     '#9370DB',
    'cf':       '#E24B4A',
    'ekf':      '#EF9F27',
    'madgwick': '#44AA66',
    'mahony':   '#378ADD',
}
_FILTERS = list(_FILTER_COLORS.keys())


class StateEstimationPanel:
    def __init__(self, shared, fig_dir, ts):
        self._sh      = shared
        self._fig_dir = fig_dir
        self._ts      = ts
        self._snap_n  = 0
        self._setup()

    def _setup(self):
        fig = plt.figure(figsize=(12, 9), facecolor='#ffffff')
        fig.canvas.manager.set_window_title('State Estimation')
        gs = gridspec.GridSpec(3, 1, figure=fig,
                               hspace=0.45, left=0.08, right=0.97, top=0.92, bottom=0.06)

        ax_roll  = fig.add_subplot(gs[0])
        ax_pitch = fig.add_subplot(gs[1])
        ax_yaw   = fig.add_subplot(gs[2])

        fig.text(0.5, 0.97, 'State Estimation',
                 color='black', fontsize=13, fontweight='bold', ha='center', va='top')

        self._lines: dict = {}
        for ax, angle, ylabel in (
            (ax_roll,  'roll',  'Roll [deg]'),
            (ax_pitch, 'pitch', 'Pitch [deg]'),
            (ax_yaw,   'yaw',   'Yaw [deg]'),
        ):
            ax.set_facecolor('#ffffff')
            ax.tick_params(colors='black', labelsize=8)
            ax.spines[['bottom', 'top', 'left', 'right']].set_color('#555')
            ax.grid(True, color='#dddddd', linewidth=0.5)
            ax.set_ylabel(ylabel, color='black', fontsize=9)
            lines = {}
            for f in _FILTERS:
                (ln,) = ax.plot([], [], color=_FILTER_COLORS[f], lw=1, label=f)
                lines[f] = ln
            ax.legend(loc='upper right', fontsize=7, framealpha=0.3,
                      labelcolor='black', facecolor='#eee', ncol=5)
            self._lines[angle] = lines

        ax_yaw.set_xlabel('Sample Number', color='black', fontsize=9)

        self.fig   = fig
        self._axes = (ax_roll, ax_pitch, ax_yaw)
        fig.canvas.mpl_connect('key_press_event', self._on_key)

    def redraw(self):
        sh = self._sh
        n  = len(sh.roll_cf_buf)
        if n < 2:
            return
        x = np.arange(n)

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
                y = np.array(buf)
                m = min(n, len(y))
                self._lines[angle][f].set_data(x[:m], y[:m])

        for ax in self._axes:
            ax.relim()
            ax.autoscale_view()

    def _on_key(self, event):
        if event.key != 's':
            return
        self._snap_n += 1
        path = self._fig_dir / f'state_snap_{self._ts}_{self._snap_n:03d}.png'
        self.fig.savefig(path, dpi=150, bbox_inches='tight', facecolor='#ffffff')
        print(f'[StateEst] Snapshot -> {path}')
