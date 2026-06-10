# panel_monitor_imu.py  —  monitor mode 창2: IMU raw (좌) + Jitter 분석 (우)
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec

TARGET_MS = 10.0
HIST_BINS = 50
_COLORS   = ['#E24B4A', '#44AA66', '#378ADD']


class MonitorImuPanel:
    def __init__(self, shared, fig_dir, ts):
        self._sh      = shared
        self._fig_dir = fig_dir
        self._ts      = ts
        self._snap_n  = 0
        self._setup()

    # ── Figure 구성 ────────────────────────────────────────────────────────────
    def _setup(self):
        fig = plt.figure(figsize=(18, 9), facecolor='#ffffff')
        fig.canvas.manager.set_window_title('Monitor — IMU + Jitter')

        outer = gridspec.GridSpec(
            1, 2, figure=fig,
            width_ratios=[1, 1.3],
            left=0.06, right=0.97,
            top=0.93,  bottom=0.07,
            wspace=0.08,
        )

        # 좌: IMU (3행)
        left_gs = gridspec.GridSpecFromSubplotSpec(3, 1, subplot_spec=outer[0], hspace=0.45)
        ax_acc = fig.add_subplot(left_gs[0])
        ax_gyr = fig.add_subplot(left_gs[1])
        ax_mag = fig.add_subplot(left_gs[2])

        # 우: Jitter (상: 시계열, 하: 히스토그램+통계)
        right_gs  = gridspec.GridSpecFromSubplotSpec(2, 1, subplot_spec=outer[1],
                                                     height_ratios=[1.2, 1], hspace=0.45)
        ax_line   = fig.add_subplot(right_gs[0])
        right_bot = gridspec.GridSpecFromSubplotSpec(1, 2, subplot_spec=right_gs[1], wspace=0.30)
        ax_hist   = fig.add_subplot(right_bot[0])
        ax_stat   = fig.add_subplot(right_bot[1])

        for ax in (ax_acc, ax_gyr, ax_mag, ax_line, ax_hist):
            ax.set_facecolor('#ffffff')
            ax.tick_params(colors='black', labelsize=8)
            ax.spines[['bottom', 'top', 'left', 'right']].set_color('#aaa')
            ax.grid(True, color='#eeeeee', linewidth=0.6)

        ax_stat.set_facecolor('#f8f8f8')
        ax_stat.axis('off')

        fig.text(0.18, 0.97, 'IMU Monitor',    color='black', fontsize=12, fontweight='bold', ha='center', va='top')
        fig.text(0.65, 0.97, 'Jitter Monitor', color='black', fontsize=12, fontweight='bold', ha='center', va='top')

        def _imu_lines(ax, labels, ylabel, x_label=False):
            lines = []
            for label, color in zip(labels, _COLORS):
                (ln,) = ax.plot([], [], color=color, lw=0.9, label=label)
                lines.append(ln)
            ax.set_ylabel(ylabel, color='black', fontsize=9)
            ax.legend(loc='upper right', fontsize=7, framealpha=0.3, labelcolor='black', facecolor='#eee', ncol=3)
            if x_label:
                ax.set_xlabel('Sample Number', color='black', fontsize=9)
            else:
                ax.tick_params(labelbottom=False)
            return lines

        self._ln_acc = _imu_lines(ax_acc, ['ax', 'ay', 'az'], 'Accel [g]')
        self._ln_gyr = _imu_lines(ax_gyr, ['gx', 'gy', 'gz'], 'Gyro [deg/s]')
        self._ln_mag = _imu_lines(ax_mag, ['mx', 'my', 'mz'], 'Mag [uT]', x_label=True)

        (self._ln_iv,) = ax_line.plot([], [], color='#378ADD', lw=1.0, label='Interval')
        ax_line.axhline(TARGET_MS, color='#E24B4A', lw=1.0, ls='--', label=f'Target {TARGET_MS} ms')
        ax_line.set_ylabel('Sample Interval [ms]', color='black', fontsize=9)
        ax_line.set_xlabel('Sample Number',        color='black', fontsize=9)
        ax_line.legend(loc='upper right', fontsize=8, framealpha=0.3, labelcolor='black', facecolor='#eee')

        ax_hist.set_xlabel('Sample Interval [ms]', color='black', fontsize=9)
        ax_hist.set_ylabel('Count',                color='black', fontsize=9)

        self._stat = ax_stat.text(
            0.05, 0.95, '',
            transform=ax_stat.transAxes,
            color='black', fontsize=10, fontfamily='monospace',
            va='top', ha='left', linespacing=2.0,
        )

        self.fig      = fig
        self._ax_acc  = ax_acc
        self._ax_gyr  = ax_gyr
        self._ax_mag  = ax_mag
        self._ax_line = ax_line
        self._ax_hist = ax_hist

        fig.canvas.mpl_connect('key_press_event', self._on_key)

    # ── 갱신 ──────────────────────────────────────────────────────────────────
    def redraw(self):
        self._redraw_imu()
        self._redraw_jitter()

    def _redraw_imu(self):
        sh = self._sh
        n  = len(sh.ax_buf)
        if n < 2:
            return
        x = np.arange(n)
        for lines, bufs in (
            (self._ln_acc, (sh.ax_buf, sh.ay_buf, sh.az_buf)),
            (self._ln_gyr, (sh.gx_buf, sh.gy_buf, sh.gz_buf)),
            (self._ln_mag, (sh.mx_buf, sh.my_buf, sh.mz_buf)),
        ):
            for ln, buf in zip(lines, bufs):
                y = np.array(buf)
                m = min(n, len(y))
                ln.set_data(x[:m], y[:m])
        for ax in (self._ax_acc, self._ax_gyr, self._ax_mag):
            ax.relim()
            ax.autoscale_view()

    def _redraw_jitter(self):
        sh = self._sh
        if len(sh.intervals) < 2:
            return
        iv = np.array(sh.intervals)
        x  = np.arange(len(iv))

        self._ln_iv.set_data(x, iv)
        for coll in self._ax_line.collections:
            coll.remove()
        self._ax_line.fill_between(x, iv, TARGET_MS, where=(iv > TARGET_MS), alpha=0.2, color='#E24B4A')
        self._ax_line.fill_between(x, iv, TARGET_MS, where=(iv < TARGET_MS), alpha=0.2, color='#44AA66')
        self._ax_line.relim()
        self._ax_line.autoscale_view()
        self._ax_line.set_ylim(max(0, TARGET_MS - 8), TARGET_MS + 8)

        self._ax_hist.cla()
        self._ax_hist.set_facecolor('#ffffff')
        self._ax_hist.tick_params(colors='black', labelsize=8)
        self._ax_hist.spines[['bottom', 'top', 'left', 'right']].set_color('#aaa')
        self._ax_hist.grid(True, color='#eeeeee', linewidth=0.6)
        self._ax_hist.hist(iv, bins=HIST_BINS, color='#378ADD', alpha=0.85, edgecolor='none')
        self._ax_hist.axvline(TARGET_MS, color='#E24B4A', lw=1.2, ls='--')
        self._ax_hist.set_xlabel('Sample Interval [ms]', color='black', fontsize=9)
        self._ax_hist.set_ylabel('Count',                color='black', fontsize=9)

        self._stat.set_text(
            f'Sample Count  {sh.total_count:>8d}\n'
            f'Drop Count    {sh.drop_count:>8d}\n'
            f'CRC Errors    {sh.crc_errors:>8d}\n'
            f'\n'
            f'mean          {np.mean(iv):>8.3f} ms\n'
            f'std           {np.std(iv):>8.3f} ms\n'
            f'min           {np.min(iv):>8.3f} ms\n'
            f'max           {np.max(iv):>8.3f} ms\n'
            f'p95           {np.percentile(iv, 95):>8.3f} ms\n'
            f'p99           {np.percentile(iv, 99):>8.3f} ms\n'
        )

    # ── 스냅샷 ('s' 키) ────────────────────────────────────────────────────────
    def _on_key(self, event):
        if event.key != 's':
            return
        self._snap_n += 1
        path = self._fig_dir / f'imu_jitter_snap_{self._ts}_{self._snap_n:03d}.png'
        self.fig.savefig(path, dpi=150, bbox_inches='tight', facecolor='#ffffff')
        print(f'[MonitorIMU] Snapshot -> {path}')
