"""
File Name: ./src/ahrs/utils/reporter.py
Author: Beomjun Chung
Updated: 2026-05-24

Description:
  필터 비교 + 노이즈 분석 결과 통합 리포트

    Sections:
        1. Filter Performance (RMSE table)
        2. Noise Analysis    (Allan variance, PSD, mag calibration)
        3. Config Summary    (trajectory, IMU model 파라미터)
"""

from __future__ import annotations

import json
from datetime import datetime
from pathlib import Path
from typing import Any

class FilterReport:
    def __init__(
        self,
        estimation_results: dict[str, dict[str, Any]],
        noise_results:  dict | None = None,
        config:         dict | None = None,
    ):
        """
        Args:
            estimation_results: {filter_name: {rmse_deg, geodesic_rmse_deg, ...}}
            noise_results:  _run_noise_stage() 반환값
                            {allan_gyro, allan_accel, mag_cal}
            config:         config dict
        """
        self._results = estimation_results
        self._noise   = noise_results or {}
        self._config  = config or {}
        self._ts      = datetime.now().strftime("%Y-%m-%d %H:%M:%S")

    # --- markdown ---
    def to_markdown(self) -> str:
        lines: list[str] = []
        lines += self._md_header()
        lines += self._md_filter_perf()
        if self._noise:
            lines += self._md_noise()
        lines += self._md_config_summary()
        return "\n".join(lines)

    def _md_header(self) -> list[str]:
        cfg_traj = self._config.get("trajectory", {}).get("type", "N/A")
        cfg_dur  = self._config.get("simulation", {}).get("duration", "N/A")
        cfg_dt   = self._config.get("simulation", {}).get("dt", "N/A")
        return [
            "# AHRS Filter Comparison Report",
            "",
            f"Generated : {self._ts}",
            f"Trajectory: {cfg_traj}  |  Duration: {cfg_dur} s  |  dt: {cfg_dt} s",
            "",
        ]

    def _md_filter_perf(self) -> list[str]:
        lines = [
            "## 1. Estimator Performance",
            "",
            "| Estimator | Roll RMSE [deg] | Pitch RMSE [deg] | Yaw RMSE [deg] | Geodesic RMSE [deg] |",
            "|--------|:---------------:|:----------------:|:--------------:|:-------------------:|",
        ]
        for name, res in self._results.items():
            rmse = res.get("rmse_deg", [0.0, 0.0, 0.0])
            geo  = res.get("geodesic_rmse_deg", 0.0)
            lines.append(
                f"| **{name}** | {rmse[0]:.3f} | {rmse[1]:.3f} | {rmse[2]:.3f} | {geo:.3f} |"
            )
        lines.append("")
        return lines

    def _md_noise(self) -> list[str]:
        lines = ["## 2. Noise Analysis", ""]

        # Allan variance — gyro
        ag = self._noise.get("allan_gyro", {})
        if ag:
            lines += [
                "### 2-1. Gyro Allan Variance",
                "",
                "| Axis | ARW [deg/√h] | BI [deg/h] | RRW [deg/h/√h] |",
                "|:----:|:------------:|:----------:|:--------------:|",
            ]
            for ax in ["x", "y", "z"]:
                arw = ag.get(f"arw_deg_per_sqrth_{ax}", float("nan"))
                bi  = ag.get(f"bi_deg_h_{ax}",          float("nan"))
                rrw = ag.get(f"rrw_deg_h_sqrth_{ax}",   float("nan"))
                lines.append(f"| {ax} | {arw:.4f} | {bi:.4f} | {rrw:.4f} |")
            lines.append("")

        # Allan variance — accel
        aa = self._noise.get("allan_accel", {})
        if aa:
            lines += [
                "### 2-2. Accel Allan Variance",
                "",
                "| Axis | VRW [deg/√h] | BI [deg/h] | RRW [deg/h/√h] |",
                "|:----:|:------------:|:----------:|:--------------:|",
            ]
            for ax in ["x", "y", "z"]:
                arw = aa.get(f"arw_deg_per_sqrth_{ax}", float("nan"))
                bi  = aa.get(f"bi_deg_h_{ax}",          float("nan"))
                rrw = aa.get(f"rrw_deg_h_sqrth_{ax}",   float("nan"))
                lines.append(f"| {ax} | {arw:.4f} | {bi:.4f} | {rrw:.4f} |")
            lines.append("")

        # Mag calibration
        mc = self._noise.get("mag_cal", {})
        if mc:
            hi  = mc.get("hard_iron_uT", [0, 0, 0])
            si  = mc.get("soft_iron_inv", [[1,0,0],[0,1,0],[0,0,1]])
            lines += [
                "### 2-3. Magnetometer Calibration",
                "",
                f"Hard iron offset [µT]: [{', '.join(f'{v:.3f}' for v in hi)}]",
                "",
                "Soft iron correction matrix (W⁻¹):",
                "```",
            ]
            for row in si:
                lines.append("  " + "  ".join(f"{v:+.4f}" for v in row))
            lines += ["```", ""]

        return lines

    def _md_config_summary(self) -> list[str]:
        cfg = self._config
        if not cfg:
            return []
        imu = cfg.get("imu", {})
        gyro = imu.get("gyroscope", {})
        accel = imu.get("accelerometer", {})
        lines = [
            "## 3. Config Summary",
            "",
            "| Parameter | Value |",
            "|-----------|-------|",
            f"| Trajectory | {cfg.get('trajectory',{}).get('type','N/A')} |",
            f"| Duration [s] | {cfg.get('simulation',{}).get('duration','N/A')} |",
            f"| dt [s] | {cfg.get('simulation',{}).get('dt','N/A')} |",
            f"| Gyro ARW [deg/√h] | {gyro.get('arw_deg_per_sqrth','N/A')} |",
            f"| Gyro BI [deg/h] | {gyro.get('bias_instability_deg_h','N/A')} |",
            f"| Accel VRW [m/s/√h] | {accel.get('vrw_m_per_s_per_sqrth','N/A')} |",
            "",
        ]
        return lines

    # --- json format report ---
    def to_json(self) -> str:
        data = {
            "generated":       self._ts,
            "filter_results":  {
                name: {k: v for k, v in res.items()
                       if not isinstance(v, __import__("numpy").ndarray)}
                for name, res in self._results.items()
            },
            "noise_results": self._noise,
        }
        return json.dumps(data, indent=2, default=str)
    
    # --- html format report ---
    def to_html(self) -> str:
        # For simplicity, we convert the markdown report to HTML using a basic approach.
        # In a real implementation, you might want to use a library like markdown2 or similar.
        md = self.to_markdown()
        html = "<html><head><meta charset='UTF-8'><title>AHRS Filter Comparison Report</title></head><body>"
        for line in md.splitlines():
            if line.startswith("# "):
                html += f"<h1>{line[2:]}</h1>"
            elif line.startswith("## "):
                html += f"<h2>{line[3:]}</h2>"
            elif line.startswith("### "):
                html += f"<h3>{line[4:]}</h3>"
            elif line.startswith("|"):
                if not hasattr(self, "_in_table"):
                    html += "<table border='1' cellspacing='0' cellpadding='5'>"
                    self._in_table = True
                html += "<tr>" + "".join(f"<th>{cell.strip()}</th>" if i == 0 else f"<td>{cell.strip()}</td>"
                                         for i, cell in enumerate(line.split("|")[1:-1])) + "</tr>"
            elif line.strip() == "":
                if hasattr(self, "_in_table"):
                    html += "</table>"
                    del self._in_table
                html += "<br>"
            else:
                html += f"<p>{line}</p>"
        if hasattr(self, "_in_table"):
            html += "</table>"
            del self._in_table
        html += "</body></html>"
        return html

    # --- save ---
    def save(self, export_dir: str | Path, format: str = "markdown") -> Path:
        export_dir = Path(export_dir)
        export_dir.mkdir(parents=True, exist_ok=True)
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        if format == "markdown":
            path = export_dir / f"report_{timestamp}.md"
            path.write_text(self.to_markdown(), encoding="utf-8")
        elif format == "json":
            path = export_dir / f"report_{timestamp}.json"
            path.write_text(self.to_json(), encoding="utf-8")
        elif format == "html":
            path = export_dir / f"report_{timestamp}.html"
            path.write_text(self.to_html(), encoding="utf-8")
        else:
            raise ValueError(f"Unknown format: {format!r}")
        return path