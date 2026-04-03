import argparse
from pathlib import Path
from analysis.utils.data_loader import load_simulation_data
from analysis.utils.plotting import plot_trajectories, plot_states_vs_time, plot_covariances


def main():
    parser = argparse.ArgumentParser(
        description="Plot UAV-UGV cooperative localization simulation outputs"
    )
    parser.add_argument(
        "--output-dir", "-d", required=True, help="Path to simulation_output/<your-subdir>"
    )
    parser.add_argument(
        "--plots",
        choices=["all", "traj", "states", "cov"],
        default="all",
        help="Which plots to generate",
    )
    parser.add_argument(
        "--no-save", action="store_true", help="Do not save HTML files (show interactively only)"
    )
    args = parser.parse_args()

    output_path = Path(args.output_dir)
    if not output_path.exists():
        print(f"Directory not found: {output_path}")
        return

    df, settings = load_simulation_data(output_path)
    if settings is not None:
        print("Filter settings:\n", settings)

    save = not args.no_save
    out_dir = output_path if save else None

    if args.plots in ("all", "traj"):
        plot_trajectories(df, str(out_dir / "trajectories.html") if out_dir else None)
    if args.plots in ("all", "states"):
        plot_states_vs_time(df, str(out_dir / "states_vs_time.html") if out_dir else None)
    if args.plots in ("all", "cov"):
        plot_covariances(df, str(out_dir / "covariances.html") if out_dir else None)

    print("Done! Interactive Plotly plots generated.")


if __name__ == "__main__":
    main()
