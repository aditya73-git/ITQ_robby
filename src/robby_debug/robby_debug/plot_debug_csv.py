import argparse
import csv
from pathlib import Path


def main() -> None:
    parser = argparse.ArgumentParser(description="Plot Robby debug CSV logs.")
    parser.add_argument("csv_path", help="Path to the debug CSV log")
    parser.add_argument(
        "--output",
        help="Optional output image path. Defaults to <csv_path stem>_plot.png",
    )
    args = parser.parse_args()

    try:
        import matplotlib.pyplot as plt
    except ImportError as error:
        raise SystemExit(
            "matplotlib is required for plotting. Install python3-matplotlib and try again."
        ) from error

    csv_path = Path(args.csv_path).expanduser().resolve()
    output_path = (
        Path(args.output).expanduser().resolve()
        if args.output
        else csv_path.with_name(f"{csv_path.stem}_plot.png")
    )

    time_s = []
    gt_x = []
    gt_y = []
    wheel_x = []
    wheel_y = []
    ekf_x = []
    ekf_y = []
    err_x = []
    err_y = []
    err_yaw = []

    with csv_path.open("r", encoding="ascii") as csv_file:
        reader = csv.DictReader(csv_file)
        for row in reader:
            time_s.append(float(row["time_s"]))
            gt_x.append(float(row["gt_x"]))
            gt_y.append(float(row["gt_y"]))
            ekf_x.append(float(row["ekf_x"]))
            ekf_y.append(float(row["ekf_y"]))
            err_x.append(float(row["err_x"])) if row["err_x"] else err_x.append(float("nan"))
            err_y.append(float(row["err_y"])) if row["err_y"] else err_y.append(float("nan"))
            err_yaw.append(float(row["err_yaw"])) if row["err_yaw"] else err_yaw.append(float("nan"))
            wheel_x.append(float(row["wheel_x"])) if row["wheel_x"] else wheel_x.append(float("nan"))
            wheel_y.append(float(row["wheel_y"])) if row["wheel_y"] else wheel_y.append(float("nan"))

    if time_s:
        start = time_s[0]
        time_s = [value - start for value in time_s]

    figure, axes = plt.subplots(2, 2, figsize=(12, 8))

    axes[0, 0].plot(gt_x, gt_y, label="Ground truth")
    axes[0, 0].plot(ekf_x, ekf_y, label="EKF")
    axes[0, 0].plot(wheel_x, wheel_y, label="Wheel odom", alpha=0.7)
    axes[0, 0].set_title("XY Trajectory")
    axes[0, 0].set_xlabel("x [m]")
    axes[0, 0].set_ylabel("y [m]")
    axes[0, 0].axis("equal")
    axes[0, 0].legend()

    axes[0, 1].plot(time_s, err_x, label="x error")
    axes[0, 1].plot(time_s, err_y, label="y error")
    axes[0, 1].set_title("Position Error")
    axes[0, 1].set_xlabel("time [s]")
    axes[0, 1].set_ylabel("error [m]")
    axes[0, 1].legend()

    axes[1, 0].plot(time_s, err_yaw, label="yaw error")
    axes[1, 0].set_title("Yaw Error")
    axes[1, 0].set_xlabel("time [s]")
    axes[1, 0].set_ylabel("error [rad]")
    axes[1, 0].legend()

    axes[1, 1].plot(time_s, gt_x, label="gt x")
    axes[1, 1].plot(time_s, ekf_x, label="ekf x")
    axes[1, 1].plot(time_s, gt_y, label="gt y")
    axes[1, 1].plot(time_s, ekf_y, label="ekf y")
    axes[1, 1].set_title("Trajectory Components")
    axes[1, 1].set_xlabel("time [s]")
    axes[1, 1].set_ylabel("position [m]")
    axes[1, 1].legend()

    figure.tight_layout()
    figure.savefig(output_path)
    print(output_path)


if __name__ == "__main__":
    main()
