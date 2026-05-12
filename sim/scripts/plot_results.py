from pathlib import Path
import sys
import numpy as np
import matplotlib.pyplot as plt

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

def load_csv(path):
    return np.genfromtxt(path, delimiter=",", names=True)

def main():
    data_dir = ROOT / "data"
    truth = load_csv(data_dir / "truth.csv")
    mahony = load_csv(data_dir / "mahony.csv")
    madgwick = load_csv(data_dir / "madgwick.csv")
    ekf = load_csv(data_dir / "ekf.csv")

    t = truth["t"]

    plt.figure(figsize=(12, 8))
    for i, angle in enumerate(["roll", "pitch", "yaw"]):
        plt.subplot(3, 1, i + 1)
        plt.plot(t, truth[f"{angle}_true_deg"], label="truth")
        plt.plot(t, mahony[f"{angle}_deg"], label="mahony")
        plt.plot(t, madgwick[f"{angle}_deg"], label="madgwick")
        plt.plot(t, ekf[f"{angle}_deg"], label="ekf")
        plt.ylabel(f"{angle} [deg]")
        plt.grid(True)
    plt.xlabel("time [s]")
    plt.legend()
    plt.tight_layout()
    plt.savefig(data_dir / "attitude_compare.png", dpi=150)

    plt.figure(figsize=(12, 8))
    for i, angle in enumerate(["roll", "pitch", "yaw"]):
        plt.subplot(3, 1, i + 1)
        plt.plot(t, mahony[f"err_{angle}_deg"], label="mahony")
        plt.plot(t, madgwick[f"err_{angle}_deg"], label="madgwick")
        plt.plot(t, ekf[f"err_{angle}_deg"], label="ekf")
        plt.ylabel(f"error {angle} [deg]")
        plt.grid(True)
    plt.xlabel("time [s]")
    plt.legend()
    plt.tight_layout()
    plt.savefig(data_dir / "attitude_error_compare.png", dpi=150)

    plt.figure(figsize=(8, 8))
    plt.scatter(truth["mx"], truth["my"], s=3, alpha=0.4)
    plt.xlabel("mx")
    plt.ylabel("my")
    plt.title("Magnetometer XY scatter (raw)")
    plt.grid(True)
    plt.tight_layout()
    plt.savefig(data_dir / "mag_xy_raw.png", dpi=150)

    print("Saved plots to data/")
    plt.show()


if __name__ == "__main__":
    main()
