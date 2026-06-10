# panel_jitter.py
# JitterPanel: serial_logger.py가 생성한 SharedState를 받아 지터 시각화.
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec

TARGET_MS = 10.0
HIST_BINS = 50


class JitterPanel:
    def __init__(self, shared, fig_dir, ts):
        self._sh      = shared
        self._fig_dir = fig_dir
        self._ts      = ts
        self._snap_n  = 0
        self._setup()

    # --- Figure 구성 --------------------------------------------------
    def _setup(self):
        fig = plt.figure(figsize=(12, 8), facecolor='#ffffff')
        fig.canvas.manager.set_window_title('Jitter Monitor')
        gs = gridspec.GridSpec(2, 2, figure=fig,
                               hspace=0.4, wspace=0.3,
                               left=0.08, right=0.97,
                               top=0.92,  bottom=0.08)

        ax_line = fig.add_subplot(gs[0, :])
        ax_hist = fig.add_subplot(gs[1, 0])
        ax_stat = fig.add_subplot(gs[1, 1])

        for ax in (ax_line, ax_hist):
            ax.set_facecolor('#ffffff')
            ax.tick_params(colors='black', labelsize=8)
            ax.spines[['bottom', 'top', 'left', 'right']].set_color('#555')
            ax.grid(True, color='#dddddd', linewidth=0.5)

        ax_stat.set_facecolor('#f5f5f5')
        ax_stat.axis('off')

        fig.text(0.5, 0.97, 'Jitter Monitor', color='black', fontsize=13, fontweight='bold', ha='center', va='top')

        (ln,) = ax_line.plot([], [], color='#378ADD', lw=1, label='Interval')
        ax_line.axhline(TARGET_MS, color='#E24B4A', lw=1, ls='--', label=f'Target {TARGET_MS} ms')
        ax_line.set_ylabel('Sample Interval [ms]', color='black', fontsize=9)
        ax_line.set_xlabel('Sample Number',         color='black', fontsize=9)
        ax_line.legend(loc='upper right', fontsize=8, framealpha=0.3, labelcolor='black', facecolor='#eee')

        ax_hist.set_xlabel('Sample Interval [ms]', color='black', fontsize=9)
        ax_hist.set_ylabel('Count',                color='black', fontsize=9)

        stat_txt = ax_stat.text(
            0.05, 0.95, '',
            transform=ax_stat.transAxes,
            color='black', fontsize=10, fontfamily='monospace',
            va='top', ha='left', linespacing=2.0,
        )

        self.fig      = fig
        self._ax_line = ax_line
        self._ax_hist = ax_hist
        self._ln      = ln
        self._stat    = stat_txt

        fig.canvas.mpl_connect('key_press_event', self._on_key)

    # ── 그래프 갱신 ────────────────────────────────────────────
    def redraw(self):
        sh = self._sh
        if len(sh.intervals) < 2:
            return

        iv = np.array(sh.intervals)
        x  = np.arange(len(iv))

        self._ln.set_data(x, iv)
        for coll in self._ax_line.collections:
            coll.remove()
        self._ax_line.fill_between(x, iv, TARGET_MS,
                                   where=(iv > TARGET_MS),
                                   alpha=0.2, color='#E24B4A')
        self._ax_line.fill_between(x, iv, TARGET_MS,
                                   where=(iv < TARGET_MS),
                                   alpha=0.2, color='#44AA66')
        self._ax_line.relim()
        self._ax_line.autoscale_view(scaley=False)
        self._ax_line.set_ylim(TARGET_MS - 1.0, TARGET_MS + 1.0)

        self._ax_hist.cla()
        self._ax_hist.set_facecolor('#ffffff')
        self._ax_hist.tick_params(colors='black', labelsize=8)
        self._ax_hist.spines[['bottom', 'top', 'left', 'right']].set_color('#555')
        self._ax_hist.grid(True, color='#dddddd', linewidth=0.5)
        self._ax_hist.hist(iv, bins=HIST_BINS,
                           color='#378ADD', alpha=0.8, edgecolor='none')
        self._ax_hist.axvline(TARGET_MS, color='#E24B4A', lw=1.2, ls='--')
        self._ax_hist.set_xlim(TARGET_MS - 1.0, TARGET_MS + 1.0)
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

    # ── 스냅샷 ('s' 키) ────────────────────────────────────────
    def _on_key(self, event):
        if event.key != 's':
            return
        self._snap_n += 1
        path = self._fig_dir / f'jitter_snap_{self._ts}_{self._snap_n:03d}.png'
        self.fig.savefig(path, dpi=150, bbox_inches='tight', facecolor='#ffffff')
        print(f'[Jitter] Snapshot -> {path}')
