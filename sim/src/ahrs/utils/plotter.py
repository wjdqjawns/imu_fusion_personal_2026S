"""
File Name: ./src/ahrs/utils/plotter.py
Author: Beomjun Chung
Updated: 2026-05-18

Description:
  Matplotlib + SciencePlots 래퍼 — IEEE/Nature 스타일 그래프 생성

    purpose:
        시뮬레이션 결과를 학술지(IEEE, Nature) 스타일로 출력.
        공통 플롯 패턴을 메서드로 제공하여 scripts에서 간결하게 사용.

    Notes:
        SciencePlots 스타일:
            'ieee'    : IEEE 저널 스타일 (단/이중 컬럼, pt 단위)
            'nature'  : Nature 스타일 (sans-serif, 컴팩트)
            'science' : 기본 과학 스타일 (serif, 깔끔)
            'no-latex': LaTeX 없이 렌더링 (LaTeX 미설치 환경)

        LaTeX 없이 사용하려면 style에 'no-latex' 추가:
            Plotter(style=['science', 'ieee', 'no-latex'])
"""

from __future__ import annotations

from contextlib import contextmanager
from pathlib import Path
from typing import Any

import numpy as np
import matplotlib
import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec

try:
    import scienceplots
    _HAS_SCIENCEPLOTS = True
except ImportError:
    _HAS_SCIENCEPLOTS = False

FILTER_COLORS = {
    "truth":         "#000000",
    "complementary": "#0072B2",
    "mahony":        "#D55E00",
    "madgwick":      "#009E73",
    "ekf":           "#CC79A7",
}

FILTER_LABELS = {
    "truth":         "Ground Truth",
    "complementary": "Complementary",
    "mahony":        "Mahony",
    "madgwick":      "Madgwick",
    "ekf":           "EKF",
}

AXIS_LABELS = ["Roll", "Pitch", "Yaw"]

@contextmanager
def science_style(style: list[str] | str):
    """SciencePlots 스타일 컨텍스트 매니저."""
    if isinstance(style, str):
        style = [style]
    if not _HAS_SCIENCEPLOTS and any(s in style for s in ["science", "ieee", "nature"]):
        # fallback: plain matplotlib
        yield
        return
    with plt.style.context(style):
        yield

class Plotter:
    def __init__(
        self,
        style: list[str] | str = None,
        dpi: int = 150,
        fmt: str = "png",
    ):
        """
        Args:
            style: SciencePlots 스타일 목록.
                   None이면 ['science', 'ieee', 'no-latex'] 자동 선택.
            dpi:   저장 해상도
            fmt:   출력 포맷 ('png', 'pdf', 'svg')
        """
        if style is None:
            style = ["science", "ieee", "no-latex"]
        self.style = [style] if isinstance(style, str) else list(style)
        self.dpi   = int(dpi)
        self.fmt   = fmt

    # ── 자세 비교: ground truth + 필터 추정값 ────────────────────────────────

    def attitude_comparison(
        self,
        t: np.ndarray,
        truth_euler_deg: np.ndarray,
        filter_results: dict[str, np.ndarray],
        out_path: Path | None = None,
        disturbance_spans: list[tuple[float, float]] | None = None,
    ) -> plt.Figure:
        """
        Roll / Pitch / Yaw 시계열 비교 플롯.

        Args:
            t:                 시간 [s], shape (N,)
            truth_euler_deg:   GT Euler [deg], shape (N, 3) [roll, pitch, yaw]
            filter_results:    {filter_name: euler_deg (N,3)}
            out_path:          저장 경로. None이면 저장 안 함.
            disturbance_spans: [(t_start, t_end), ...] 외란 구간 음영 표시
        """
        with science_style(self.style):
            fig, axes = plt.subplots(3, 1, figsize=(6.5, 5.5), sharex=True)

            for i, ax in enumerate(axes):
                # Ground truth
                ax.plot(t, truth_euler_deg[:, i],
                        color=FILTER_COLORS["truth"],
                        linestyle="--", linewidth=0.8,
                        label=FILTER_LABELS["truth"], zorder=10)

                # 필터 추정값
                for name, euler in filter_results.items():
                    ax.plot(t, euler[:, i],
                            color=FILTER_COLORS.get(name, None),
                            linewidth=0.8,
                            label=FILTER_LABELS.get(name, name))

                # 외란 구간 음영
                if disturbance_spans:
                    for t0, t1 in disturbance_spans:
                        ax.axvspan(t0, t1, alpha=0.15, color="red", linewidth=0)

                ax.set_ylabel(f"{AXIS_LABELS[i]} [°]")
                ax.grid(True)

            axes[-1].set_xlabel("Time [s]")
            axes[0].legend(loc="upper right", fontsize=6, ncol=2)
            fig.suptitle("Attitude Comparison", y=1.01)
            fig.tight_layout()

            if out_path:
                fig.savefig(out_path, dpi=self.dpi, bbox_inches="tight")

        return fig

    # ── 자세 오차: 각 필터별 ─────────────────────────────────────────────────

    def attitude_error(
        self,
        t: np.ndarray,
        filter_errors: dict[str, np.ndarray],
        out_path: Path | None = None,
        disturbance_spans: list[tuple[float, float]] | None = None,
    ) -> plt.Figure:
        """
        Roll / Pitch / Yaw 오차 시계열.

        Args:
            filter_errors: {filter_name: euler_error_deg (N,3)}
        """
        with science_style(self.style):
            fig, axes = plt.subplots(3, 1, figsize=(6.5, 5.5), sharex=True)

            for i, ax in enumerate(axes):
                ax.axhline(0, color="black", linewidth=0.5, linestyle=":")
                for name, err in filter_errors.items():
                    ax.plot(t, err[:, i],
                            color=FILTER_COLORS.get(name, None),
                            linewidth=0.7,
                            label=FILTER_LABELS.get(name, name))

                if disturbance_spans:
                    for t0, t1 in disturbance_spans:
                        ax.axvspan(t0, t1, alpha=0.15, color="red", linewidth=0)

                ax.set_ylabel(f"Δ{AXIS_LABELS[i]} [°]")
                ax.grid(True)

            axes[-1].set_xlabel("Time [s]")
            axes[0].legend(loc="upper right", fontsize=6, ncol=2)
            fig.suptitle("Attitude Error", y=1.01)
            fig.tight_layout()

            if out_path:
                fig.savefig(out_path, dpi=self.dpi, bbox_inches="tight")

        return fig

    # ── Geodesic 오차 ────────────────────────────────────────────────────────

    def geodesic_error(
        self,
        t: np.ndarray,
        geo_errors: dict[str, np.ndarray],
        out_path: Path | None = None,
        disturbance_spans: list[tuple[float, float]] | None = None,
    ) -> plt.Figure:
        """
        Geodesic 오차 시계열 [deg].

        Args:
            geo_errors: {filter_name: error_deg (N,)}
        """
        with science_style(self.style):
            fig, ax = plt.subplots(figsize=(6.5, 2.8))

            for name, err in geo_errors.items():
                ax.plot(t, err,
                        color=FILTER_COLORS.get(name, None),
                        linewidth=0.8,
                        label=FILTER_LABELS.get(name, name))

            if disturbance_spans:
                for t0, t1 in disturbance_spans:
                    ax.axvspan(t0, t1, alpha=0.15, color="red", linewidth=0,
                               label="Disturbance" if t0 == disturbance_spans[0][0] else "")

            ax.set_xlabel("Time [s]")
            ax.set_ylabel("Geodesic Error [°]")
            ax.legend(loc="upper right", fontsize=7)
            ax.grid(True)
            fig.tight_layout()

            if out_path:
                fig.savefig(out_path, dpi=self.dpi, bbox_inches="tight")

        return fig

    # ── RMSE 바 차트 ────────────────────────────────────────────────────────

    def rmse_bar(
        self,
        rmse_table: dict[str, dict[str, float]],
        out_path: Path | None = None,
    ) -> plt.Figure:
        """
        필터별 RMSE 바 차트.

        Args:
            rmse_table: {filter_name: {roll, pitch, yaw, geodesic} [deg]}
        """
        with science_style(self.style):
            names   = list(rmse_table.keys())
            metrics = ["roll", "pitch", "yaw", "geodesic"]
            x       = np.arange(len(metrics))
            width   = 0.8 / len(names)

            fig, ax = plt.subplots(figsize=(5, 3))

            for i, name in enumerate(names):
                vals = [rmse_table[name].get(m, 0.0) for m in metrics]
                ax.bar(x + i * width - 0.4 + width / 2, vals, width,
                       label=FILTER_LABELS.get(name, name),
                       color=FILTER_COLORS.get(name, None))

            ax.set_xticks(x)
            ax.set_xticklabels(["Roll", "Pitch", "Yaw", "Geodesic"])
            ax.set_ylabel("RMSE [°]")
            ax.legend(fontsize=7)
            ax.grid(True, axis="y")
            fig.tight_layout()

            if out_path:
                fig.savefig(out_path, dpi=self.dpi, bbox_inches="tight")

        return fig

    # ── 3D trajectory overlay ────────────────────────────────────────────────

    def trajectory_3d(
        self,
        truth_euler_deg: np.ndarray,
        filter_results: dict[str, np.ndarray],
        out_path: Path | None = None,
    ) -> plt.Figure:
        """
        2-panel: (left) Roll-Pitch-Yaw 3D, (right) body-Z on unit sphere.

        Args:
            truth_euler_deg:  GT Euler [deg], shape (N, 3)  [roll, pitch, yaw]
            filter_results:   {filter_name: euler_deg (N, 3)}
        """
        from ahrs.core.orientation.transforms import Transforms

        def _body_z(euler_deg: np.ndarray) -> np.ndarray:
            """Euler angles [deg] (N,3) -> body Z-axis in world frame (N,3)."""
            out = np.empty_like(euler_deg)
            for i, (r, p, y) in enumerate(euler_deg):
                R = Transforms.euler_to_dcm(
                    np.radians(r), np.radians(p), np.radians(y)
                )
                out[i] = R[:, 2]
            return out

        def _sphere_wireframe(ax, alpha: float = 0.08):
            u = np.linspace(0, 2 * np.pi, 30)
            v = np.linspace(0, np.pi, 20)
            xs = np.outer(np.cos(u), np.sin(v))
            ys = np.outer(np.sin(u), np.sin(v))
            zs = np.outer(np.ones_like(u), np.cos(v))
            ax.plot_wireframe(xs, ys, zs, color="gray", alpha=alpha, linewidth=0.3, zorder=0)

        with science_style(self.style):
            fig = plt.figure(figsize=(9, 4.0))
            ax_euler = fig.add_subplot(121, projection="3d")
            ax_sph   = fig.add_subplot(122, projection="3d")

            # ── Left: Euler angle 3D ─────────────────────────────────────────
            ax_euler.plot(*truth_euler_deg.T,
                          color=FILTER_COLORS["truth"],
                          linestyle="--", linewidth=0.8,
                          label=FILTER_LABELS["truth"], zorder=10)
            for name, euler in filter_results.items():
                ax_euler.plot(*euler.T,
                              color=FILTER_COLORS.get(name, "#888888"),
                              linewidth=0.6,
                              label=FILTER_LABELS.get(name, name))

            ax_euler.set_xlabel("Roll [deg]",  fontsize=7, labelpad=4)
            ax_euler.set_ylabel("Pitch [deg]", fontsize=7, labelpad=4)
            ax_euler.set_zlabel("Yaw [deg]",   fontsize=7, labelpad=8)
            ax_euler.zaxis.set_rotate_label(False)
            ax_euler.view_init(elev=20, azim=45)
            ax_euler.tick_params(labelsize=5)
            ax_euler.set_title("Euler Angle Trajectory", fontsize=8, pad=4)
            ax_euler.legend(fontsize=5, loc="upper left")

            # ── Right: body-Z on unit sphere ─────────────────────────────────
            _sphere_wireframe(ax_sph)

            # NED axis indicators (N=+x red, E=+y green, D=+z blue)
            for vec, lbl, col in [
                ([1, 0, 0], "N", "#E41A1C"),
                ([0, 1, 0], "E", "#4DAF4A"),
                ([0, 0, 1], "D", "#377EB8"),
            ]:
                ax_sph.quiver(0, 0, 0, *vec,
                              length=1.3, arrow_length_ratio=0.15,
                              color=col, linewidth=1.0, zorder=5)
                ax_sph.text(vec[0]*1.45, vec[1]*1.45, vec[2]*1.45,
                            lbl, fontsize=6, color=col,
                            ha="center", va="center")

            bz_truth = _body_z(truth_euler_deg)
            ax_sph.plot(*bz_truth.T,
                        color=FILTER_COLORS["truth"],
                        linestyle="--", linewidth=0.9,
                        label=FILTER_LABELS["truth"], zorder=10)

            for name, euler in filter_results.items():
                bz = _body_z(euler)
                ax_sph.plot(*bz.T,
                            color=FILTER_COLORS.get(name, "#888888"),
                            linewidth=0.6,
                            label=FILTER_LABELS.get(name, name))

            ax_sph.set_xlim(-1.2, 1.2)
            ax_sph.set_ylim(-1.2, 1.2)
            ax_sph.set_zlim(-1.2, 1.2)
            ax_sph.set_xlabel("N (x)",  fontsize=7, labelpad=4)
            ax_sph.set_ylabel("E (y)",  fontsize=7, labelpad=4)
            ax_sph.set_zlabel("D (z)",  fontsize=7, labelpad=8)
            ax_sph.zaxis.set_rotate_label(False)
            ax_sph.view_init(elev=20, azim=45)
            ax_sph.tick_params(labelsize=5)
            ax_sph.set_title("Body Z-axis on Unit Sphere (NED)", fontsize=8, pad=4)
            ax_sph.legend(fontsize=5, loc="upper left")

            fig.tight_layout()

            if out_path:
                fig.savefig(out_path, dpi=self.dpi, bbox_inches="tight")

        return fig

    # ── Allan variance ────────────────────────────────────────────────────────

    def allan_variance(
        self,
        tau: np.ndarray,
        adev: np.ndarray,
        axis_labels: list[str] | None = None,
        out_path: Path | None = None,
    ) -> plt.Figure:
        """
        Allan deviation 곡선. log-log 스케일.

        Args:
            tau:          [s], shape (M,)
            adev:         shape (M,) or (M, 3)
            axis_labels:  축별 범례 레이블
        """
        with science_style(self.style):
            fig, ax = plt.subplots(figsize=(5, 3.5))

            if adev.ndim == 1:
                ax.loglog(tau, adev, linewidth=0.9)
            else:
                colors = ["#0072B2", "#D55E00", "#009E73"]
                labels = axis_labels or ["x", "y", "z"]
                for i in range(adev.shape[1]):
                    ax.loglog(tau, adev[:, i],
                              color=colors[i % len(colors)],
                              linewidth=0.9, label=labels[i])
                ax.legend(fontsize=7)

            # 기울기 가이드라인
            tau_mid = np.sqrt(tau[0] * tau[-1])
            adev_mid = np.nanmedian(adev) if adev.ndim == 1 else np.nanmedian(adev[:, 0])
            for slope, label in [(-0.5, "−½ ARW"), (0.0, "0  BI"), (0.5, "+½ RRW")]:
                yvals = adev_mid * (tau / tau_mid) ** slope
                ax.loglog(tau, yvals, "k:", linewidth=0.5, alpha=0.4)
                ax.text(tau[-1] * 0.9, yvals[-1], label, fontsize=5, alpha=0.6, va="center")

            ax.set_xlabel("Averaging Time τ [s]")
            ax.set_ylabel("Allan Deviation")
            ax.grid(True, which="both", alpha=0.3)
            fig.tight_layout()

            if out_path:
                fig.savefig(out_path, dpi=self.dpi, bbox_inches="tight")

        return fig

    # ── 수렴 비교 ─────────────────────────────────────────────────────────────

    def convergence(
        self,
        t: np.ndarray,
        geo_errors: dict[str, np.ndarray],
        threshold_deg: float = 2.0,
        zoom_end_s: float = 20.0,
        out_path: Path | None = None,
    ) -> plt.Figure:
        """
        초기 수렴 구간 확대 플롯.

        Args:
            zoom_end_s:     확대 구간 끝 [s]
            threshold_deg:  수렴 기준선 [deg]
        """
        mask = t <= zoom_end_s

        with science_style(self.style):
            fig, ax = plt.subplots(figsize=(5, 3))

            for name, err in geo_errors.items():
                ax.plot(t[mask], err[mask],
                        color=FILTER_COLORS.get(name, None),
                        linewidth=0.8,
                        label=FILTER_LABELS.get(name, name))

            ax.axhline(threshold_deg, color="gray", linewidth=0.6,
                       linestyle="--", label=f"{threshold_deg}° threshold")

            ax.set_xlabel("Time [s]")
            ax.set_ylabel("Geodesic Error [°]")
            ax.legend(fontsize=7)
            ax.grid(True)
            fig.tight_layout()

            if out_path:
                fig.savefig(out_path, dpi=self.dpi, bbox_inches="tight")

        return fig

    # ── PSD (Welch) ───────────────────────────────────────────────────────────

    def psd(
        self,
        data: np.ndarray,
        fs: float,
        title: str = "PSD",
        axis_labels: list[str] | None = None,
        unit_label: str = "unit²/Hz",
        out_path: Path | None = None,
    ) -> plt.Figure:
        """
        Welch PSD 플롯 (semilogy).

        Args:
            data:        (N,) 또는 (N, 3) 시계열
            fs:          샘플링 주파수 [Hz]
            unit_label:  y축 단위 레이블
        """
        from scipy.signal import welch

        if data.ndim == 1:
            data = data[:, np.newaxis]
        n_axes  = data.shape[1]
        colors  = ["#0072B2", "#D55E00", "#009E73"]
        labels  = (axis_labels or ["x", "y", "z"])[:n_axes]
        nperseg = min(len(data) // 8, 4096)

        with science_style(self.style):
            fig, axes = plt.subplots(n_axes, 1,
                                     figsize=(5.5, 1.9 * n_axes), sharex=True)
            if n_axes == 1:
                axes = [axes]

            for i, ax in enumerate(axes):
                freqs, psd_vals = welch(data[:, i], fs=fs, nperseg=nperseg)
                # DC 제외
                ax.semilogy(freqs[1:], psd_vals[1:],
                            color=colors[i % len(colors)],
                            linewidth=0.8, label=labels[i])
                ax.set_ylabel(f"[{unit_label}]", fontsize=6)
                ax.legend(fontsize=6, loc="upper right")
                ax.grid(True, which="both", alpha=0.3)

            axes[-1].set_xlabel("Frequency [Hz]")
            fig.suptitle(title, y=1.01)
            fig.tight_layout()

            if out_path:
                fig.savefig(out_path, dpi=self.dpi, bbox_inches="tight")

        return fig

    # ── Mag hard/soft iron 보정 비교 ──────────────────────────────────────────

    def mag_sphere(
        self,
        mag_raw: np.ndarray,
        mag_cal: np.ndarray,
        out_path: Path | None = None,
    ) -> plt.Figure:
        """
        Hard/soft iron 보정 전후 비교 플롯.

        Layout: 2×2
            [3D scatter]        [XY 투영 + 공분산 타원]
            [XZ 투영 + 타원]    [YZ 투영 + 타원]

        공분산 타원 해석:
            Raw  (주황) — 중심 오프셋 = hard iron, 타원 찌그러짐 = soft iron
            Cal. (파랑) — 중심 ≈ 원점, 원 형태 = 보정 완료

        Args:
            mag_raw: 보정 전 자기장 [uT], shape (N, 3)
            mag_cal: 보정 후 자기장 [uT], shape (N, 3)
        """
        from matplotlib.patches import Ellipse

        RAW_C = "#D55E00"   # 주황 = Raw (hard/soft iron 포함)
        CAL_C = "#0072B2"   # 파랑 = Calibrated
        s_kw  = dict(s=1.5, alpha=0.30, linewidths=0)

        def _cov_ellipse(ax, data2d: np.ndarray, color: str,
                         n_std: float = 1.5) -> None:
            """
            n_std σ 공분산 타원 오버레이.
            중심(+)과 타원 외곽선을 파선으로 표시.
            """
            if len(data2d) < 4:
                return
            m   = data2d.mean(axis=0)
            cov = np.cov(data2d.T)
            ev, ec = np.linalg.eigh(cov)
            # 내림차순 정렬 (장축 먼저)
            idx = ev.argsort()[::-1]
            ev, ec = ev[idx], ec[:, idx]
            ev = np.maximum(ev, 0.0)   # 수치 오차 방지
            angle = np.degrees(np.arctan2(ec[1, 0], ec[0, 0]))
            w = 2.0 * n_std * np.sqrt(ev[0])
            h = 2.0 * n_std * np.sqrt(ev[1])
            patch = Ellipse(m, w, h, angle=angle,
                            edgecolor=color, facecolor="none",
                            linewidth=1.1, linestyle="--", zorder=5)
            ax.add_patch(patch)
            ax.plot(*m, "+", color=color, markersize=8,
                    markeredgewidth=1.5, zorder=6)

        with science_style(self.style):
            fig = plt.figure(figsize=(7.5, 6.5))
            gs  = matplotlib.gridspec.GridSpec(
                2, 2, figure=fig, hspace=0.42, wspace=0.35
            )
            ax3d  = fig.add_subplot(gs[0, 0], projection="3d")
            ax_xy = fig.add_subplot(gs[0, 1])
            ax_xz = fig.add_subplot(gs[1, 0])
            ax_yz = fig.add_subplot(gs[1, 1])

            # ── 3D scatter ──────────────────────────────────────────────────
            ax3d.scatter(*mag_raw.T, c=RAW_C,
                         label="Raw (uncalibrated)", **s_kw)
            ax3d.scatter(*mag_cal.T, c=CAL_C,
                         label="Calibrated", **s_kw)
            ax3d.set_xlabel("X [µT]", fontsize=6)
            ax3d.set_ylabel("Y [µT]", fontsize=6)
            ax3d.set_zlabel("Z [µT]", fontsize=6)
            ax3d.legend(fontsize=6, loc="upper left",
                        markerscale=4, handletextpad=0.4)
            ax3d.tick_params(labelsize=5)

            raw_r = np.linalg.norm(mag_raw, axis=1)
            cal_r = np.linalg.norm(mag_cal, axis=1)
            ax3d.set_title(
                f"3D Scatter\n"
                f"|m| raw: {raw_r.mean():.1f}±{raw_r.std():.2f} µT  "
                f"cal: {cal_r.mean():.1f}±{cal_r.std():.2f} µT",
                fontsize=6,
            )

            # ── 2D 투영 + 공분산 타원 ────────────────────────────────────────
            # 타원이 scatter 범위 안에 들어오도록 축 여유 계산
            all_2d_data = [
                (mag_raw[:, [0, 1]], mag_cal[:, [0, 1]]),
                (mag_raw[:, [0, 2]], mag_cal[:, [0, 2]]),
                (mag_raw[:, [1, 2]], mag_cal[:, [1, 2]]),
            ]
            for ax, (xi, yi), xl, yl, ttl, (raw_2d, cal_2d) in zip(
                [ax_xy, ax_xz, ax_yz],
                [(0, 1), (0, 2), (1, 2)],
                ["X [µT]", "X [µT]", "Y [µT]"],
                ["Y [µT]", "Z [µT]", "Z [µT]"],
                ["XY projection", "XZ projection", "YZ projection"],
                all_2d_data,
            ):
                # 산점도
                ax.scatter(raw_2d[:, 0], raw_2d[:, 1],
                           c=RAW_C, label="Raw (uncalibrated)", **s_kw)
                ax.scatter(cal_2d[:, 0], cal_2d[:, 1],
                           c=CAL_C, label="Calibrated", **s_kw)

                # 공분산 타원 (1.5σ, 파선)
                _cov_ellipse(ax, raw_2d, RAW_C, n_std=1.5)
                _cov_ellipse(ax, cal_2d, CAL_C, n_std=1.5)

                ax.set_xlabel(xl, fontsize=6)
                ax.set_ylabel(yl, fontsize=6)
                ax.set_aspect("equal", adjustable="datalim")
                ax.grid(True, alpha=0.3)
                ax.tick_params(labelsize=5)
                ax.set_title(ttl, fontsize=7)

                legend = ax.legend(fontsize=5.5, loc="upper right",
                                   markerscale=4, handletextpad=0.3,
                                   framealpha=0.8)
                # 타원 범례 항목 추가 (파선)
                from matplotlib.lines import Line2D
                legend_extras = [
                    Line2D([0], [0], color=RAW_C, ls="--", lw=1.1,
                           label="Raw 1.5$\\sigma$ ellipse"),
                    Line2D([0], [0], color=CAL_C, ls="--", lw=1.1,
                           label="Cal. 1.5$\\sigma$ ellipse"),
                ]
                ax.legend(
                    handles=list(legend.legend_handles) + legend_extras,
                    fontsize=5.0, loc="upper right",
                    framealpha=0.8, handlelength=1.5,
                )

            fig.suptitle("Magnetometer Hard/Soft Iron Calibration", y=1.01)

            if out_path:
                fig.savefig(out_path, dpi=self.dpi, bbox_inches="tight")

        return fig

    # ── 저장 헬퍼 ─────────────────────────────────────────────────────────────

    def savefig(self, fig: plt.Figure, path: Path) -> None:
        path = Path(path)
        path.parent.mkdir(parents=True, exist_ok=True)
        fig.savefig(path, dpi=self.dpi, bbox_inches="tight")
        plt.close(fig)

    @classmethod
    def for_ieee(cls, dpi: int = 300) -> "Plotter":
        """IEEE 저널 스타일 인스턴스."""
        return cls(style=["science", "ieee", "no-latex"], dpi=dpi)

    @classmethod
    def for_nature(cls, dpi: int = 300) -> "Plotter":
        """Nature 스타일 인스턴스."""
        return cls(style=["science", "nature", "no-latex"], dpi=dpi)
