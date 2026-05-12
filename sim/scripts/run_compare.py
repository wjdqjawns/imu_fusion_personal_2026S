from pathlib import Path
import sys
import yaml

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from src.simulation import simulate_imu
from src.runner import run_filters, save_results

def main():
    cfg_path = ROOT / "config" / "default.yaml"
    with cfg_path.open("r", encoding="utf-8") as f:
        cfg = yaml.safe_load(f)

    sim = simulate_imu(cfg["simulation"])
    results = run_filters(sim, cfg)
    save_results(sim, results, ROOT / "data")

    print("Saved results to data/")
    for name, res in results.items():
        rr, pp, yy = res["rmse_deg"]
        print(f"{name:10s} RMSE [deg] -> roll={rr:.3f}, pitch={pp:.3f}, yaw={yy:.3f}")


if __name__ == "__main__":
    main()
