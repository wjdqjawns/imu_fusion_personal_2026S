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
    "ekf_euler":     "#E69F00",
    "ekf_quat":      "#CC79A7",
    "eskf":          "#56B4E9",
}

FILTER_LABELS = {
    "truth":         "Ground Truth",
    "complementary": "Complementary",
    "mahony":        "Mahony",
    "madgwick":      "Madgwick",
    "ekf":           "EKF (Quat)",
    "ekf_euler":     "EKF (Euler)",
    "ekf_quat":      "EKF (Quat)",
    "eskf":          "ESKF",
}

FILTER_LINESTYLES = {
    "complementary": (0, (5, 1)),        # loosely dashed
    "mahony":        (0, (3, 1, 1, 1)), # dash-dot
    "madgwick":      (0, (1, 1)),        # dotted
    "ekf":           "-",
    "ekf_euler":     "--",
    "ekf_quat":      "-.",
    "eskf":          "-",
}

AXIS_LABELS = ["Roll", "Pitch", "Yaw"]

@contextmanager
def science_style(style: list[str] | str):
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

    # --- raw time series plot ---
    def time_series(
        self,
        t: np.ndarray,
        data: np.ndarray,
        title: str = "",
        axis_labels: list[str] | None = None,
        out_path: Path | None = None,
    ) -> plt.Figure:
        """
        시계열 플롯 (예: 센서 노이즈).

        Args:
            t:           시간 [s], shape (N,)
            data:        시계열 데이터, shape (N,) or (N, 3)
            title:       그래프 제목
            axis_labels: 축별 범례 레이블
        """
        with science_style(self.style):
            fig, ax = plt.subplots(figsize=(6.0, 3.5))
            if data.ndim == 1:
                ax.plot(t, data, linewidth=0.9)
            else:
                colors = ["#0072B2", "#D55E00", "#009E73"]
                labels = axis_labels or ["x", "y", "z"]
                for i in range(data.shape[1]):
                    ax.plot(t, data[:, i],
                            color=colors[i % len(colors)],
                            linewidth=0.9, label=labels[i])
                ax.legend(fontsize=7)

            ax.set_xlabel("Time [s]")
            ax.set_ylabel("Value")
            ax.set_title(title)
            ax.grid(True)
            fig.tight_layout()

            if out_path:
                fig.savefig(out_path, dpi=self.dpi, bbox_inches="tight")

        return fig

    # --- compare path estimation results: ground truth + estimation value ---
    def attitude_comparison(
        self,
        t: np.ndarray,
        truth_euler_deg: np.ndarray,
        estimation_results: dict[str, np.ndarray],
        out_path: Path | None = None,
        disturbance_spans: list[tuple[float, float]] | None = None,
        traj_type: str = "",
    ) -> plt.Figure:
        """
        roll, pitch, yaw angle in world frame comparison.

        Args:
            t:                 시간 [s], shape (N,)
            truth_euler_deg:   GT Euler [deg], shape (N, 3) [roll, pitch, yaw]
            estimation_results: {filter_name: euler_deg (N,3)}
            traj_type:         궤적 종류 문자열 (제목에 표시)
        """
        with science_style(self.style):
            fig, axes = plt.subplots(3, 1, figsize=(7.0, 6.0), sharex=True)

            for i, ax in enumerate(axes):
                # Ground truth — 굵고 뚜렷하게
                ax.plot(t, truth_euler_deg[:, i],
                        color=FILTER_COLORS["truth"],
                        linestyle="--", linewidth=1.6,
                        label=FILTER_LABELS["truth"], zorder=10)

                # 필터별 추정 결과 — 색상 + 선 스타일 조합
                for name, euler in estimation_results.items():
                    ax.plot(t, euler[:, i],
                            color=FILTER_COLORS.get(name, "#888888"),
                            linestyle=FILTER_LINESTYLES.get(name, "-"),
                            linewidth=1.0,
                            label=FILTER_LABELS.get(name, name))

                # 외란 구간 음영
                if disturbance_spans:
                    for k, (t0, t1) in enumerate(disturbance_spans):
                        ax.axvspan(t0, t1, alpha=0.12, color="red", linewidth=0,
                                   label="Disturbance" if (i == 0 and k == 0) else "")

                ax.set_ylabel(f"{AXIS_LABELS[i]} [°]", fontsize=8)
                ax.grid(True, alpha=0.35)

            axes[-1].set_xlabel("Time [s]")
            axes[0].legend(loc="upper right", fontsize=6, ncol=3,
                           framealpha=0.85)
            title = "Attitude Comparison"
            if traj_type:
                title += f"  [{traj_type}]"
            fig.suptitle(title, y=1.01)
            fig.tight_layout()

            if out_path:
                fig.savefig(out_path, dpi=self.dpi, bbox_inches="tight")

        return fig

    # --- attitude estimation error (based on world frame)---
    def attitude_error(
        self,
        t: np.ndarray,
        filter_errors: dict[str, np.ndarray],
        out_path: Path | None = None,
        disturbance_spans: list[tuple[float, float]] | None = None,
    ) -> plt.Figure:
        """
        roll, pitch, yaw angle estimation error each estimator.

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

    # --- Geodesic Error ---
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

    # --- estimation rmse chart ---
    def rmse_bar(
        self,
        rmse_table: dict[str, dict[str, float]],
        out_path: Path | None = None,
    ) -> plt.Figure:
        """
        rmse chart each estimation results.

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

    # ── 공통 헬퍼 ─────────────────────────────────────────────────────────────
    @staticmethod
    def _body_z(euler_deg: np.ndarray) -> np.ndarray:
        """Euler [deg] (N,3) → body Z-axis 방향 (N,3), world frame."""
        from ahrs.utils.transforms import Transforms
        out = np.empty_like(euler_deg)
        for i, (r, p, y) in enumerate(euler_deg):
            R = Transforms.euler_to_dcm(np.radians(r), np.radians(p), np.radians(y))
            out[i] = R[:, 2]
        return out

    @staticmethod
    def _sphere_wireframe(ax, alpha: float = 0.04, n_lines: int = 18):
        """단위 구 와이어프레임. alpha를 낮춰 궤적이 앞에 보이게."""
        u = np.linspace(0, 2 * np.pi, n_lines)
        v = np.linspace(0, np.pi, n_lines // 2)
        xs = np.outer(np.cos(u), np.sin(v))
        ys = np.outer(np.sin(u), np.sin(v))
        zs = np.outer(np.ones_like(u), np.cos(v))
        ax.plot_wireframe(xs, ys, zs, color="#aaaaaa", alpha=alpha,
                          linewidth=0.25, zorder=0)

    # --- 3D Euler Angle Trajectory ---
    def euler_trajectory_3d(
        self,
        truth_euler_deg: np.ndarray,
        filter_results: dict[str, np.ndarray],
        traj_type: str = "",
        out_path: Path | None = None,
    ) -> plt.Figure:
        """
        Roll-Pitch-Yaw 3D 궤적 단독 그래프.

        Args:
            truth_euler_deg: GT Euler [deg], shape (N, 3)
            filter_results:  {filter_name: euler_deg (N, 3)}
            traj_type:       궤적 종류 문자열 (제목 표시용)
        """
        with science_style(self.style):
            fig = plt.figure(figsize=(6.0, 5.5))
            ax = fig.add_subplot(111, projection="3d")

            # GT — 굵고 뚜렷하게
            ax.plot(*truth_euler_deg.T,
                    color=FILTER_COLORS["truth"],
                    linestyle="--", linewidth=2.0,
                    label=FILTER_LABELS["truth"], zorder=10)
            # 시작점 / 끝점 마커
            ax.scatter(*truth_euler_deg[0],  color="lime",  s=60,
                       marker="o", zorder=20, label="Start")
            ax.scatter(*truth_euler_deg[-1], color="red",   s=60,
                       marker="*", zorder=20, label="End")

            for name, euler in filter_results.items():
                ax.plot(*euler.T,
                        color=FILTER_COLORS.get(name, "#888888"),
                        linestyle=FILTER_LINESTYLES.get(name, "-"),
                        linewidth=1.0, alpha=0.85,
                        label=FILTER_LABELS.get(name, name))

            ax.set_xlabel("Roll [°]",  fontsize=7, labelpad=4)
            ax.set_ylabel("Pitch [°]", fontsize=7, labelpad=4)
            ax.set_zlabel("Yaw [°]",   fontsize=7, labelpad=8)
            ax.zaxis.set_rotate_label(False)
            ax.view_init(elev=22, azim=50)
            ax.tick_params(labelsize=5)
            title = "Euler Angle Trajectory"
            if traj_type:
                title += f"\n[{traj_type}]"
            ax.set_title(title, fontsize=8, pad=6)
            ax.legend(fontsize=5, loc="upper left", framealpha=0.8)
            fig.tight_layout()

            if out_path:
                fig.savefig(out_path, dpi=self.dpi, bbox_inches="tight")

        return fig

    # --- Body Z-axis on Unit Sphere ---
    def body_axis_sphere(
        self,
        truth_euler_deg: np.ndarray,
        filter_results: dict[str, np.ndarray],
        traj_type: str = "",
        out_path: Path | None = None,
    ) -> plt.Figure:
        """
        Body Z-축 방향의 단위 구면 궤적.
        GT는 시간 순서를 색상(viridis)으로 표현.

        Args:
            truth_euler_deg: GT Euler [deg], shape (N, 3)
            filter_results:  {filter_name: euler_deg (N, 3)}
            traj_type:       궤적 종류 문자열 (제목 표시용)
        """
        bz_truth = self._body_z(truth_euler_deg)
        n = len(bz_truth)

        with science_style(self.style):
            fig = plt.figure(figsize=(6.5, 6.0))
            ax = fig.add_subplot(111, projection="3d")

            # 구면 와이어프레임 (매우 연하게)
            self._sphere_wireframe(ax, alpha=0.04)

            # NED 축 화살표
            for vec, lbl, col in [
                ([1, 0, 0], "N", "#E41A1C"),
                ([0, 1, 0], "E", "#4DAF4A"),
                ([0, 0, 1], "D", "#377EB8"),
            ]:
                ax.quiver(0, 0, 0, *vec,
                          length=1.25, arrow_length_ratio=0.18,
                          color=col, linewidth=1.5, zorder=5)
                ax.text(vec[0]*1.38, vec[1]*1.38, vec[2]*1.38,
                        lbl, fontsize=8, fontweight="bold", color=col,
                        ha="center", va="center")

            # GT 궤적 — 시간 진행을 viridis 색으로 표현 (두꺼운 선)
            cmap = plt.cm.viridis
            colors_t = cmap(np.linspace(0, 1, n))
            for i in range(n - 1):
                ax.plot(bz_truth[i:i+2, 0],
                        bz_truth[i:i+2, 1],
                        bz_truth[i:i+2, 2],
                        color=colors_t[i], linewidth=2.0, alpha=0.9, zorder=10)

            # GT 시작 / 끝 마커
            ax.scatter(*bz_truth[0],  color=cmap(0.0), s=80,
                       marker="o", edgecolors="k", linewidths=0.8,
                       zorder=20, label="GT Start")
            ax.scatter(*bz_truth[-1], color=cmap(1.0), s=80,
                       marker="*", edgecolors="k", linewidths=0.8,
                       zorder=20, label="GT End")

            # 필터 추정 궤적
            for name, euler in filter_results.items():
                bz = self._body_z(euler)
                ax.plot(*bz.T,
                        color=FILTER_COLORS.get(name, "#888888"),
                        linestyle=FILTER_LINESTYLES.get(name, "-"),
                        linewidth=1.3, alpha=0.75,
                        label=FILTER_LABELS.get(name, name),
                        zorder=8)

            # 색상바 (GT 시간 진행 표시)
            sm = plt.cm.ScalarMappable(cmap=cmap,
                                        norm=plt.Normalize(0, 1))
            sm.set_array([])
            cbar = fig.colorbar(sm, ax=ax, pad=0.1, shrink=0.55,
                                orientation="vertical")
            cbar.set_label("GT time (0→1)", fontsize=7)
            cbar.ax.tick_params(labelsize=6)

            ax.set_xlim(-1.15, 1.15)
            ax.set_ylim(-1.15, 1.15)
            ax.set_zlim(-1.15, 1.15)
            ax.set_xlabel("N (x)", fontsize=7, labelpad=4)
            ax.set_ylabel("E (y)", fontsize=7, labelpad=4)
            ax.set_zlabel("D (z)", fontsize=7, labelpad=8)
            ax.zaxis.set_rotate_label(False)
            ax.view_init(elev=25, azim=40)
            ax.tick_params(labelsize=5)
            title = "Body Z-axis on Unit Sphere (NED)"
            if traj_type:
                title += f"\n[{traj_type}]"
            ax.set_title(title, fontsize=8, pad=6)
            ax.legend(fontsize=5, loc="upper left", framealpha=0.85,
                      ncol=2)
            fig.tight_layout()

            if out_path:
                fig.savefig(out_path, dpi=self.dpi, bbox_inches="tight")

        return fig

    # 하위 호환: 기존 코드가 trajectory_3d를 호출할 경우 euler 3D만 저장
    def trajectory_3d(
        self,
        truth_euler_deg: np.ndarray,
        filter_results: dict[str, np.ndarray],
        traj_type: str = "",
        out_path: Path | None = None,
    ) -> plt.Figure:
        return self.euler_trajectory_3d(truth_euler_deg, filter_results,
                                        traj_type=traj_type, out_path=out_path)

    # --- Allan variance ---
    def allan_variance(
        self,
        tau: np.ndarray,
        adev: np.ndarray,
        axis_labels: list[str] | None = None,
        out_path: Path | None = None,
    ) -> plt.Figure:
        """
        Allan deviation 곡선. log-log scale.

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

            # slope guideline
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

    # --- convergence comparison ---
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

    # ── magnetometer hard/soft iron distortion correction comparison ---
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
    
    # ── FFT spectrum ───────────────────────────────────────────────────────────
    def fft_spectrum(
        self,
        data: np.ndarray,
        fs: float,
        title: str = "FFT Spectrum",
        axis_labels: list[str] | None = None,
        unit_label: str = "amplitude",
        out_path: Path | None = None,
    ) -> plt.Figure:
        """
        단측 FFT 스펙트럼 (semilogy).

        Args:
            data:   (N,) 또는 (N, 3) 시계열
            fs:     샘플링 주파수 [Hz]
        """
        if data.ndim == 1:
            data = data[:, np.newaxis]
        n_axes = data.shape[1]
        n      = data.shape[0]
        freqs  = np.fft.rfftfreq(n, d=1.0 / fs)
        colors = ["#0072B2", "#D55E00", "#009E73"]
        labels = (axis_labels or ["x", "y", "z"])[:n_axes]

        with science_style(self.style):
            fig, axes = plt.subplots(n_axes, 1,
                                     figsize=(5.5, 1.9 * n_axes), sharex=True)
            if n_axes == 1:
                axes = [axes]

            for i, ax in enumerate(axes):
                mag = np.abs(np.fft.rfft(data[:, i])) / n
                ax.semilogy(freqs[1:], mag[1:],
                            color=colors[i % len(colors)],
                            linewidth=0.7, label=labels[i])
                ax.set_ylabel(f"|FFT| [{unit_label}]", fontsize=6)
                ax.legend(fontsize=6, loc="upper right")
                ax.grid(True, which="both", alpha=0.3)

            axes[-1].set_xlabel("Frequency [Hz]")
            fig.suptitle(title, y=1.01)
            fig.tight_layout()

            if out_path:
                fig.savefig(out_path, dpi=self.dpi, bbox_inches="tight")

        return fig

    # ── Frequency sweep error ──────────────────────────────────────────────────
    def freq_sweep_error(
        self,
        freq_rmse: dict[float, dict[str, float]],
        out_path: Path | None = None,
    ) -> plt.Figure:
        """
        주파수별 Geodesic RMSE 곡선.

        Args:
            freq_rmse: {freq_hz: {filter_name: geodesic_rmse_deg}}
        """
        freqs   = sorted(freq_rmse.keys())
        filters = list(next(iter(freq_rmse.values())).keys())

        with science_style(self.style):
            fig, ax = plt.subplots(figsize=(5.5, 3.5))

            for name in filters:
                vals = [freq_rmse[f].get(name, float("nan")) for f in freqs]
                ax.plot(freqs, vals,
                        color=FILTER_COLORS.get(name, None),
                        marker="o", markersize=3.5, linewidth=0.9,
                        label=FILTER_LABELS.get(name, name))

            ax.set_xscale("log")
            ax.set_xlabel("Frequency [Hz]")
            ax.set_ylabel("Geodesic RMSE [°]")
            ax.legend(fontsize=7)
            ax.grid(True, which="both", alpha=0.3)
            fig.suptitle("Frequency Sweep — Filter Error vs. Input Frequency", y=1.01)
            fig.tight_layout()

            if out_path:
                fig.savefig(out_path, dpi=self.dpi, bbox_inches="tight")

        return fig

    # ── GT vs Estimated scatter ────────────────────────────────────────────────
    def gt_vs_est_scatter(
        self,
        truth_euler_deg: np.ndarray,
        filter_euler: dict[str, np.ndarray],
        out_path: Path | None = None,
        subsample: int = 10,
    ) -> plt.Figure:
        """
        GT vs. Estimated angle 산점도 (roll / pitch / yaw 분리).

        Args:
            truth_euler_deg: (N, 3) [deg]
            filter_euler:    {name: (N, 3) [deg]}
            subsample:       시각화 속도를 위해 N점 중 1/subsample만 사용
        """
        idx = np.arange(0, len(truth_euler_deg), max(1, subsample))
        gt  = truth_euler_deg[idx]

        with science_style(self.style):
            n_filt = len(filter_euler)
            fig, axes = plt.subplots(3, n_filt,
                                     figsize=(3.2 * n_filt, 7.5),
                                     sharex="row", sharey="row")
            if n_filt == 1:
                axes = np.array(axes).reshape(3, 1)

            for j, (name, euler) in enumerate(filter_euler.items()):
                est = euler[idx]
                for i, axis_lbl in enumerate(AXIS_LABELS):
                    ax = axes[i, j]
                    lo = min(gt[:, i].min(), est[:, i].min())
                    hi = max(gt[:, i].max(), est[:, i].max())
                    margin = (hi - lo) * 0.05 + 1.0
                    diag = [lo - margin, hi + margin]

                    ax.plot(diag, diag, "k--", linewidth=0.7, alpha=0.5, zorder=0,
                            label="Ideal (y=x)")
                    ax.scatter(gt[:, i], est[:, i],
                               c=FILTER_COLORS.get(name, "#888888"),
                               s=2, alpha=0.3, linewidths=0, zorder=1)

                    if i == 0:
                        ax.set_title(FILTER_LABELS.get(name, name), fontsize=8)
                    if j == 0:
                        ax.set_ylabel(f"Est. {axis_lbl} [°]", fontsize=7)
                    ax.set_xlabel(f"GT {axis_lbl} [°]", fontsize=7)
                    ax.set_xlim(diag)
                    ax.set_ylim(diag)
                    ax.set_aspect("equal", adjustable="box")
                    ax.grid(True, alpha=0.25)
                    ax.tick_params(labelsize=5)

            fig.suptitle("GT vs. Estimated Attitude", y=1.01)
            fig.tight_layout()

            if out_path:
                fig.savefig(out_path, dpi=self.dpi, bbox_inches="tight")

        return fig

    # --- save helper ---
    def savefig(self, fig: plt.Figure, path: Path) -> None:
        path = Path(path)
        path.parent.mkdir(parents=True, exist_ok=True)
        fig.savefig(path, dpi=self.dpi, bbox_inches="tight")
        plt.close(fig)

    @classmethod
    def from_config(cls, cfg: dict) -> "Plotter":
        """cfg["export"]["fig"] 섹션에서 Plotter 인스턴스 생성."""
        export_cfg = cfg.get("export") or cfg.get("output", {})
        fig_cfg = export_cfg.get("fig", {})
        return cls(
            dpi=int(fig_cfg.get("dpi", 150)),
            fmt=fig_cfg.get("format", "png"),
        )

    @classmethod
    def for_ieee(cls, dpi: int = 300) -> "Plotter":
        """IEEE 저널 스타일 인스턴스."""
        return cls(style=["science", "ieee", "no-latex"], dpi=dpi)

    @classmethod
    def for_nature(cls, dpi: int = 300) -> "Plotter":
        """Nature 스타일 인스턴스."""
        return cls(style=["science", "nature", "no-latex"], dpi=dpi)
