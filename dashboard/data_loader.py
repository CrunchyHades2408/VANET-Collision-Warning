
from pathlib import Path
import pandas as pd

# --------------------------------------------------
# PROJECT PATHS
# --------------------------------------------------

# dashboard/data_loader.py -> dashboard -> project root
PROJECT_ROOT = Path(__file__).resolve().parent.parent

PHASE5_DIR = (
    PROJECT_ROOT / "phase5_collision_detection" / "results"
)

PHASE4_DIR = (
    PROJECT_ROOT / "omnetpp" / "veins_integration" / "results"
)

# --------------------------------------------------
# CSV FILE LOCATIONS
# --------------------------------------------------

DATA_FILES = {
    "vehicle_state": PHASE5_DIR / "vehicle_state_log.csv",
    "collision_detection": PHASE5_DIR / "collision_detection_log.csv",
    "warning_transmission": PHASE5_DIR / "warning_transmission_log.csv",
    "ground_truth": PHASE5_DIR / "ground_truth_evaluations.csv",
    "scenario_metrics": PHASE5_DIR / "phase5_scenario_evaluation_metrics.csv",

    "phase4_metrics": PHASE4_DIR / "veins_experiment_metrics.csv",
    "network_trace": PHASE4_DIR / "veins_aodv_network_trace.csv",
    "mobility_sync": PHASE4_DIR / "veins_mobility_sync.csv",
}

# --------------------------------------------------
# CSV LOADER
# --------------------------------------------------

def load_csv(file_path):
    """
    Read a CSV file safely.
    Returns an empty DataFrame if the file is missing
    or cannot be read.
    """

    if not file_path.exists():
        print(f"Missing file: {file_path}")
        return pd.DataFrame()

    try:
        df = pd.read_csv(file_path)

        # Remove accidental whitespace in column names
        df.columns = df.columns.str.strip()

        return df

    except Exception as error:
        print(f"Could not read {file_path.name}: {error}")
        return pd.DataFrame()


# --------------------------------------------------
# LOAD ALL PROJECT DATA
# --------------------------------------------------

def load_all_data():
    """
    Loads all available Phase 4 and Phase 5 datasets.
    """

    data = {}

    for name, path in DATA_FILES.items():
        data[name] = load_csv(path)

    return data


# --------------------------------------------------
# LOAD EXISTING RESULT IMAGES
# --------------------------------------------------

def get_result_images():
    """
    Returns the available Phase 5 PNG graph paths.
    """

    if not PHASE5_DIR.exists():
        return []

    return sorted(PHASE5_DIR.glob("*.png"))


# --------------------------------------------------
# DATA SUMMARY
# --------------------------------------------------

def get_data_summary(data):
    """
    Returns basic dataset counts for dashboard KPIs.
    """

    summary = {}

    summary["vehicles"] = (
        data["vehicle_state"]["vehicle_id"].nunique()
        if not data["vehicle_state"].empty
        and "vehicle_id" in data["vehicle_state"].columns
        else 0
    )

    summary["scenarios"] = (
        data["scenario_metrics"]["Scenario"].nunique()
        if not data["scenario_metrics"].empty
        and "Scenario" in data["scenario_metrics"].columns
        else 0
    )

    summary["warnings_generated"] = (
        int(data["scenario_metrics"]["WarningsGenerated"].sum())
        if not data["scenario_metrics"].empty
        and "WarningsGenerated" in data["scenario_metrics"].columns
        else 0
    )

    summary["warnings_delivered"] = (
        int(data["scenario_metrics"]["WarningsDelivered"].sum())
        if not data["scenario_metrics"].empty
        and "WarningsDelivered" in data["scenario_metrics"].columns
        else 0
    )

    return summary
