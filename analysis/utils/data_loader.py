from pathlib import Path
import pandas as pd


def _find_file(output_dir: Path, patterns: list[str]) -> Path | None:
    for pattern in patterns:
        files = list(output_dir.glob(pattern))
        if files:
            return files[0]
    return None


def load_simulation_data(output_dir: str | Path) -> tuple[pd.DataFrame, pd.DataFrame | None]:
    """Load the main simulation CSV and optional filter settings CSV from a simulation_output
    sub-directory.

    Args:
        output_dir: subdirectory name for desired plotting data under simulation_output/
    """
    output_dir = Path(output_dir)
    if not output_dir.exists():
        raise FileNotFoundError(f"Output directory not found: {output_dir}")

    # Auto-detect the main data CSV (works with ekf/ukf_simulation_data.csv or time_history.csv)
    data_file = _find_file(output_dir, ["*data*.csv", "*time*.csv"])
    if not data_file:
        raise FileNotFoundError(f"No data CSV found in {output_dir}")

    print(f"Loading simulation data from: {data_file}")
    df = pd.read_csv(data_file)

    # Auto-detect settings CSV
    settings_file = _find_file(output_dir, ["*settings*.csv", "*filter*.csv"])
    settings_df = None
    if settings_file:
        print(f"✅ Loading filter settings from: {settings_file}")
        settings_df = pd.read_csv(settings_file)

    print(f"   Columns available: {list(df.columns)}")
    return df, settings_df
