"""
Combine and aggregate benchmark results from all benchmark suite runs.
Outputs thesis-ready CSV tables, a Markdown summary, and figures.

Usage:
    python scripts/combine_benchmark_results.py
"""

from __future__ import annotations

import json
import re
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd

# ---------------------------------------------------------------------------
# Config
# ---------------------------------------------------------------------------

ROOT = Path(__file__).parent.parent
SUITE_DIR = ROOT / "test_data" / "benchmark_suite"
OUT_DIR = ROOT / "test_data" / "benchmark_results"
FIG_DIR = OUT_DIR / "figures"

PLANNER_LABELS: dict[str, str] = {
    "curobo":          "cuRobo",
    "cumotion":        "cuMotion",
    "ompl_rrtStar":    "OMPL RRT*",
    "ompl_rrtConnect": "OMPL RRTConnect",
    "hybrid":          "Hybrid",
    "ompl":            "OMPL",
}

SIMPLE_PLANNER_ORDER = ["curobo", "cumotion", "ompl_rrtStar", "ompl_rrtConnect", "hybrid"]
PNP_PLANNER_ORDER    = ["curobo", "cumotion", "ompl", "hybrid"]
HYBRID_PROFILES      = ["early", "medium", "late", "very_late"]

PHASES = [
    "pre_grasp", "grasp", "post_grasp_lift", "pre_drop",
    "release", "post_release_retreat", "return_home",
]

# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def mean_std(s: pd.Series) -> str:
    m = s.mean()
    if pd.isna(m):
        return "N/A"
    std = s.std(ddof=1) if len(s) > 1 else 0.0
    return f"{m:.3f} ± {std:.3f}"


def fmt_pct(v: float) -> str:
    return f"{v:.1f}%"


def pivot_mean_std(df: pd.DataFrame, value_col: str,
                   index: str, columns: str,
                   planner_order: list[str]) -> pd.DataFrame:
    """Return a pivot table with 'mean ± std' strings."""
    def _ms(s: pd.Series) -> str:
        if len(s) < 2:
            return f"{s.mean():.3f}"
        return mean_std(s)

    piv = df.pivot_table(values=value_col, index=index,
                         columns=columns, aggfunc=_ms)
    ordered_cols = [p for p in planner_order if p in piv.columns]
    piv = piv[ordered_cols]
    piv.columns = [PLANNER_LABELS.get(c, c) for c in piv.columns]
    piv.index.name = "Case"
    return piv


def pivot_pct(df: pd.DataFrame, value_col: str,
              index: str, columns: str,
              planner_order: list[str]) -> pd.DataFrame:
    """Return a pivot table with percentage strings."""
    piv = df.pivot_table(values=value_col, index=index,
                         columns=columns, aggfunc=lambda s: fmt_pct(s.mean() * 100))
    ordered_cols = [p for p in planner_order if p in piv.columns]
    piv = piv[ordered_cols]
    piv.columns = [PLANNER_LABELS.get(c, c) for c in piv.columns]
    piv.index.name = "Case"
    return piv


def df_to_markdown(df: pd.DataFrame) -> str:
    lines = []
    if isinstance(df.index, pd.MultiIndex):
        index_names = " / ".join(str(n) for n in df.index.names)
    else:
        index_names = df.index.name or "Index"
    header = "| " + index_names + " | " + " | ".join(str(c) for c in df.columns) + " |"
    sep    = "| " + "--- |" * (len(df.columns) + 1)
    lines.append(header)
    lines.append(sep)
    for idx, row in df.iterrows():
        idx_str = " / ".join(str(i) for i in idx) if isinstance(idx, tuple) else str(idx)
        lines.append("| " + idx_str + " | " + " | ".join(str(v) for v in row) + " |")
    return "\n".join(lines)


# ---------------------------------------------------------------------------
# Data loaders
# ---------------------------------------------------------------------------

def discover_suite_dirs() -> list[Path]:
    dirs = sorted(p for p in SUITE_DIR.iterdir()
                  if p.is_dir() and re.match(r"\d{8}_\d{6}", p.name))
    if not dirs:
        raise FileNotFoundError(f"No benchmark suite directories found in {SUITE_DIR}")
    print(f"Found {len(dirs)} suite runs: {[d.name for d in dirs]}")
    return dirs


def load_simple_benchmark(suite_dirs: list[Path]) -> pd.DataFrame:
    rows = []
    for suite in suite_dirs:
        results_dir = suite / "simple_benchmark" / "results"
        if not results_dir.exists():
            continue
        for planner_dir in results_dir.iterdir():
            if not planner_dir.is_dir():
                continue
            planner = planner_dir.name
            for csv_file in planner_dir.glob("*.csv"):
                try:
                    df = pd.read_csv(csv_file)
                    df["planner"] = planner
                    df["suite_run"] = suite.name
                    rows.append(df)
                except Exception as e:
                    print(f"  Warning: could not read {csv_file}: {e}")
    if not rows:
        return pd.DataFrame()
    out = pd.concat(rows, ignore_index=True)
    out["success_bool"] = out["success"].astype(str).str.lower() == "true"
    return out


def load_pick_and_place(suite_dirs: list[Path]) -> pd.DataFrame:
    rows = []
    for suite in suite_dirs:
        results_dir = suite / "pick_and_place" / "results"
        if not results_dir.exists():
            continue
        for csv_file in results_dir.glob("*.csv"):
            planner = csv_file.stem
            try:
                df = pd.read_csv(csv_file)
                df["planner"] = planner
                df["suite_run"] = suite.name
                rows.append(df)
            except Exception as e:
                print(f"  Warning: could not read {csv_file}: {e}")
    if not rows:
        return pd.DataFrame()
    out = pd.concat(rows, ignore_index=True)
    bool_cols: dict[str, pd.Series] = {}
    for phase in PHASES:
        col = f"{phase}_success"
        if col in out.columns:
            bool_cols[f"{phase}_success_bool"] = out[col].astype(str).str.lower() == "true"
    if bool_cols:
        bool_df = pd.DataFrame(bool_cols, index=out.index)
        bool_df["overall_success_bool"] = bool_df.all(axis=1)
        out = pd.concat([out, bool_df], axis=1)
    return out


def load_hybrid_planner(suite_dirs: list[Path]) -> pd.DataFrame:
    rows = []
    for suite in suite_dirs:
        results_dir = suite / "hybrid_planner" / "results"
        if not results_dir.exists():
            continue
        for json_file in results_dir.rglob("*.json"):
            try:
                with open(json_file) as f:
                    d = json.load(f)
                repl = d.get("replanning", {})
                obs = d.get("obstacle", {})
                row = {
                    "suite_run":     suite.name,
                    "case":          d.get("case"),
                    "profile":       obs.get("spawn_trigger_profile"),
                    "success":       bool(d.get("success", False)),
                    "timed_out":     bool(d.get("timed_out", False)),
                    "wall_time_s":   d.get("benchmark_goal_wall_time_sec"),
                    "obstacle_spawned": bool(obs.get("spawned", False)),
                    "path_invalidated": repl.get("path_invalidated_count", 0) > 0,
                    "replan_count":  repl.get("replan_count", 0),
                    "global_plan_count": repl.get("global_plan_count", 0),
                    "obstacle_to_replan_s": repl.get(
                        "time_from_obstacle_spawn_to_next_global_plan_sec"),
                    "obstacle_to_invalidation_s": repl.get(
                        "time_from_obstacle_spawn_to_path_invalidation_sec"),
                    "robot_clearance_at_spawn_m": obs.get("robot_clearance_at_spawn_m"),
                }
                rows.append(row)
            except Exception as e:
                print(f"  Warning: could not read {json_file}: {e}")
    return pd.DataFrame(rows) if rows else pd.DataFrame()


# ---------------------------------------------------------------------------
# Aggregators
# ---------------------------------------------------------------------------

def aggregate_simple_benchmark(df: pd.DataFrame) -> dict[str, pd.DataFrame]:
    if df.empty:
        return {}

    tables = {}

    # T1: Success rate
    tables["simple_success_rate"] = pivot_pct(
        df, "success_bool", "case_name", "planner", SIMPLE_PLANNER_ORDER)

    # T2: Planning time
    tables["simple_planning_time"] = pivot_mean_std(
        df, "planning_time_s", "case_name", "planner", SIMPLE_PLANNER_ORDER)

    # T3: Execution time
    tables["simple_execution_time"] = pivot_mean_std(
        df, "execution_time_s", "case_name", "planner", SIMPLE_PLANNER_ORDER)

    # T4: TCP movement
    tables["simple_tcp_movement"] = pivot_mean_std(
        df, "movement_tcp_movement_cm", "case_name", "planner", SIMPLE_PLANNER_ORDER)

    return tables


def aggregate_pick_and_place(df: pd.DataFrame) -> dict[str, pd.DataFrame]:
    if df.empty:
        return {}

    tables: dict[str, pd.DataFrame] = {}

    # P1: Success rate + most common failed phase
    p1_rows = []
    for planner, grp in df.groupby("planner"):
        success_rate = grp["overall_success_bool"].mean() * 100
        # Find the phase with the lowest success rate (= most commonly failed)
        phase_rates = {p: grp[f"{p}_success_bool"].mean()
                       for p in PHASES if f"{p}_success_bool" in grp.columns}
        worst_phase = min(phase_rates, key=phase_rates.get) if phase_rates else "N/A"
        p1_rows.append({
            "Planner":                  PLANNER_LABELS.get(planner, planner),
            "Success rate":             fmt_pct(success_rate),
            "Most common failed phase": worst_phase,
        })
    p1 = pd.DataFrame(p1_rows).set_index("Planner")
    tables["pnp_success_rate"] = p1

    # P2: Workflow timing
    p2_rows = []
    total_time_col = "total_time(s)" if "total_time(s)" in df.columns else "total_time_s"
    for planner, grp in df.groupby("planner"):
        p2_rows.append({
            "Planner":               PLANNER_LABELS.get(planner, planner),
            "Total time":            mean_std(grp[total_time_col].dropna()) + " s",
            "Planning time":         mean_std(grp["total_planning_time_s"].dropna()) + " s",
            "Execution time":        mean_std(grp["total_execution_time_s"].dropna()) + " s",
        })
    tables["pnp_workflow_timing"] = pd.DataFrame(p2_rows).set_index("Planner")

    # P3: Per-phase planning time
    p3_rows = []
    for planner, grp in df.groupby("planner"):
        row = {"Planner": PLANNER_LABELS.get(planner, planner)}
        for phase in PHASES:
            col = f"{phase}_planning_time_s"
            if col in grp.columns:
                row[phase] = mean_std(grp[col].dropna()) + " s"
            else:
                row[phase] = "N/A"
        p3_rows.append(row)
    tables["pnp_phase_planning_time"] = pd.DataFrame(p3_rows).set_index("Planner")

    # P4: Per-phase success rate (numeric, for heatmap)
    p4_rows = []
    for planner, grp in df.groupby("planner"):
        row = {"Planner": PLANNER_LABELS.get(planner, planner)}
        for phase in PHASES:
            col = f"{phase}_success_bool"
            row[phase] = grp[col].mean() * 100 if col in grp.columns else float("nan")
        p4_rows.append(row)
    tables["pnp_phase_success_rate"] = pd.DataFrame(p4_rows).set_index("Planner")

    return tables


def aggregate_hybrid_planner(df: pd.DataFrame) -> dict[str, pd.DataFrame]:
    if df.empty:
        return {}

    rows = []
    for (case, profile), grp in df.groupby(["case", "profile"]):
        replanned = grp[grp["replan_count"] > 0]
        replan_success_rate = (
            (replanned["success"].sum() / len(replanned) * 100)
            if len(replanned) > 0 else float("nan")
        )
        obstacle_to_replan = grp["obstacle_to_replan_s"].dropna()
        rows.append({
            "Case":                        case,
            "Spawn profile":               profile,
            "Success rate":                fmt_pct(grp["success"].mean() * 100),
            "Obstacle spawned rate":       fmt_pct(grp["obstacle_spawned"].mean() * 100),
            "Path invalidated rate":       fmt_pct(grp["path_invalidated"].mean() * 100),
            "Replan success rate":         fmt_pct(replan_success_rate) if not np.isnan(replan_success_rate) else "N/A",
            "Mean replan count":           f"{grp['replan_count'].mean():.2f}",
            "Mean obstacle-to-replan (s)": f"{obstacle_to_replan.mean():.3f}" if len(obstacle_to_replan) > 0 else "N/A",
            "Mean robot clearance at spawn (m)": f"{grp['robot_clearance_at_spawn_m'].dropna().mean():.3f}",
        })

    df_out = pd.DataFrame(rows).set_index(["Case", "Spawn profile"])
    return {"hybrid_obstacle": df_out}


def aggregate_resource_usage(simple_df: pd.DataFrame, pnp_df: pd.DataFrame) -> dict[str, pd.DataFrame]:
    rows = []

    def _add(df: pd.DataFrame, source: str) -> None:
        if df.empty:
            return
        planner_col = "planner"
        for planner, grp in df.groupby(planner_col):
            rows.append({
                "planner": planner,
                "source":  source,
                "ram_avg_mib": grp["planner_ram_memory_avg_mib"].mean(),
                "ram_max_mib": grp["planner_ram_memory_max_mib"].max(),
                "gpu_avg_mib": grp["planner_gpu_memory_avg_mib"].mean(),
                "gpu_max_mib": grp["planner_gpu_memory_max_mib"].max(),
            })

    _add(simple_df, "simple")
    _add(pnp_df,    "pick_and_place")

    if not rows:
        return {}

    combined = pd.DataFrame(rows)
    out_rows = []
    for planner, grp in combined.groupby("planner"):
        out_rows.append({
            "Planner":      PLANNER_LABELS.get(planner, planner),
            "RAM avg MiB":  f"{grp['ram_avg_mib'].mean():.0f}",
            "RAM max MiB":  f"{grp['ram_max_mib'].max():.0f}",
            "GPU avg MiB":  f"{grp['gpu_avg_mib'].mean():.0f}",
            "GPU max MiB":  f"{grp['gpu_max_mib'].max():.0f}",
        })
    return {"resource_usage": pd.DataFrame(out_rows).set_index("Planner")}


# ---------------------------------------------------------------------------
# Figures
# ---------------------------------------------------------------------------

PLANNER_COLORS = {
    "cuRobo":          "#1f77b4",
    "cuMotion":        "#ff7f0e",
    "OMPL RRT*":       "#2ca02c",
    "OMPL RRTConnect": "#d62728",
    "Hybrid":          "#9467bd",
    "OMPL":            "#8c564b",
}


def fig_grouped_bar(df_raw: pd.DataFrame, value_col: str, title: str,
                    ylabel: str, out_path: Path,
                    planner_order: list[str]) -> None:
    """Grouped bar chart with error bars, one group per case."""
    cases    = sorted(df_raw["case_name"].unique()) if "case_name" in df_raw else []
    planners = [p for p in planner_order if p in df_raw["planner"].unique()]
    if not cases or not planners:
        return

    n_cases   = len(cases)
    n_planners = len(planners)
    x = np.arange(n_cases)
    width = 0.8 / n_planners

    fig, ax = plt.subplots(figsize=(max(8, n_cases * 1.5), 5))
    for i, planner in enumerate(planners):
        label = PLANNER_LABELS.get(planner, planner)
        grp = df_raw[df_raw["planner"] == planner]
        means, stds = [], []
        for case in cases:
            c_grp = grp[grp["case_name"] == case][value_col].dropna()
            means.append(c_grp.mean() if len(c_grp) > 0 else 0)
            stds.append(c_grp.std(ddof=1) if len(c_grp) > 1 else 0)
        offset = (i - n_planners / 2 + 0.5) * width
        color  = PLANNER_COLORS.get(label, None)
        ax.bar(x + offset, means, width, yerr=stds, label=label,
               color=color, capsize=3, alpha=0.85)

    ax.set_xticks(x)
    ax.set_xticklabels(cases, rotation=25, ha="right", fontsize=8)
    ax.set_ylabel(ylabel)
    ax.set_title(title)
    ax.legend(loc="upper right", fontsize=8)
    ax.yaxis.grid(True, alpha=0.4)
    ax.set_axisbelow(True)
    fig.tight_layout()
    fig.savefig(out_path, dpi=150)
    plt.close(fig)
    print(f"  Saved figure: {out_path.name}")


def fig_success_rate_bar(df_raw: pd.DataFrame, out_path: Path,
                         planner_order: list[str]) -> None:
    cases    = sorted(df_raw["case_name"].unique()) if "case_name" in df_raw else []
    planners = [p for p in planner_order if p in df_raw["planner"].unique()]
    if not cases or not planners:
        return

    n_cases    = len(cases)
    n_planners = len(planners)
    x = np.arange(n_cases)
    width = 0.8 / n_planners

    fig, ax = plt.subplots(figsize=(max(8, n_cases * 1.5), 5))
    for i, planner in enumerate(planners):
        label = PLANNER_LABELS.get(planner, planner)
        grp = df_raw[df_raw["planner"] == planner]
        rates = []
        for case in cases:
            c_grp = grp[grp["case_name"] == case]["success_bool"]
            rates.append(c_grp.mean() * 100 if len(c_grp) > 0 else 0)
        offset = (i - n_planners / 2 + 0.5) * width
        color  = PLANNER_COLORS.get(label, None)
        ax.bar(x + offset, rates, width, label=label, color=color, alpha=0.85)

    ax.set_xticks(x)
    ax.set_xticklabels(cases, rotation=25, ha="right", fontsize=8)
    ax.set_ylim(0, 110)
    ax.set_ylabel("Success rate (%)")
    ax.set_title("Simple Motion – Success Rate by Case and Planner")
    ax.legend(loc="lower right", fontsize=8)
    ax.yaxis.grid(True, alpha=0.4)
    ax.set_axisbelow(True)
    fig.tight_layout()
    fig.savefig(out_path, dpi=150)
    plt.close(fig)
    print(f"  Saved figure: {out_path.name}")


def fig_pnp_phase_heatmap(pnp_phase_success: pd.DataFrame, out_path: Path) -> None:
    if pnp_phase_success.empty:
        return
    data = pnp_phase_success.astype(float)
    fig, ax = plt.subplots(figsize=(len(data.columns) * 1.4, len(data) * 0.8 + 1))
    im = ax.imshow(data.values, aspect="auto", cmap="RdYlGn", vmin=0, vmax=100)
    ax.set_xticks(range(len(data.columns)))
    ax.set_xticklabels(data.columns, rotation=30, ha="right", fontsize=8)
    ax.set_yticks(range(len(data.index)))
    ax.set_yticklabels(data.index, fontsize=9)
    for r in range(data.shape[0]):
        for c in range(data.shape[1]):
            val = data.values[r, c]
            if not np.isnan(val):
                ax.text(c, r, f"{val:.0f}%", ha="center", va="center", fontsize=8,
                        color="black" if 30 < val < 80 else "white")
    plt.colorbar(im, ax=ax, label="Success rate (%)")
    ax.set_title("Pick-and-Place – Per-Phase Success Rate")
    fig.tight_layout()
    fig.savefig(out_path, dpi=150)
    plt.close(fig)
    print(f"  Saved figure: {out_path.name}")


# ---------------------------------------------------------------------------
# Output writer
# ---------------------------------------------------------------------------

def write_outputs(all_tables: dict[str, pd.DataFrame]) -> None:
    OUT_DIR.mkdir(parents=True, exist_ok=True)
    FIG_DIR.mkdir(parents=True, exist_ok=True)

    md_sections: list[str] = ["# Benchmark Results\n"]

    section_headers = {
        "simple_success_rate":       "## Simple Motion – Table 1: Success Rate",
        "simple_planning_time":      "## Simple Motion – Table 2: Planning Time (mean ± std)",
        "simple_execution_time":     "## Simple Motion – Table 3: Execution Time (mean ± std)",
        "simple_tcp_movement":       "## Simple Motion – Table 4: TCP Movement cm (mean ± std)",
        "pnp_success_rate":          "## Pick-and-Place – Table 1: Success Rate",
        "pnp_workflow_timing":       "## Pick-and-Place – Table 2: Workflow Timing (mean ± std)",
        "pnp_phase_planning_time":   "## Pick-and-Place – Table 3: Per-Phase Planning Time (mean ± std)",
        "pnp_phase_success_rate":    "## Pick-and-Place – Table 4: Per-Phase Success Rate (%)",
        "hybrid_obstacle":           "## Hybrid Obstacle Benchmark",
        "resource_usage":            "## Resource Usage",
    }

    for key, df in all_tables.items():
        if df.empty:
            continue
        csv_path = OUT_DIR / f"{key}.csv"
        df.to_csv(csv_path)
        print(f"  Saved: {csv_path.name}")

        header = section_headers.get(key, f"## {key}")
        md_sections.append(f"\n{header}\n")
        # Format numeric percentage tables for markdown display
        if key == "pnp_phase_success_rate":
            md_df = df.map(lambda v: f"{v:.1f}%" if pd.notna(v) else "N/A")
        else:
            md_df = df
        md_sections.append(df_to_markdown(md_df))

    md_path = OUT_DIR / "tables.md"
    md_path.write_text("\n".join(md_sections))
    print(f"  Saved: {md_path.name}")


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def main() -> None:
    print("=== Benchmark Result Combiner ===\n")

    suite_dirs = discover_suite_dirs()

    print("\nLoading data...")
    simple_df = load_simple_benchmark(suite_dirs)
    pnp_df    = load_pick_and_place(suite_dirs)
    hybrid_df = load_hybrid_planner(suite_dirs)

    print(f"  Simple benchmark rows : {len(simple_df)}")
    print(f"  Pick-and-place rows   : {len(pnp_df)}")
    print(f"  Hybrid planner runs   : {len(hybrid_df)}")

    print("\nAggregating tables...")
    all_tables: dict[str, pd.DataFrame] = {}
    all_tables.update(aggregate_simple_benchmark(simple_df))
    all_tables.update(aggregate_pick_and_place(pnp_df))
    all_tables.update(aggregate_hybrid_planner(hybrid_df))
    all_tables.update(aggregate_resource_usage(simple_df, pnp_df))

    print("\nWriting outputs...")
    write_outputs(all_tables)

    print("\nGenerating figures...")
    if not simple_df.empty:
        fig_grouped_bar(
            simple_df, "planning_time_s",
            "Simple Motion – Planning Time by Case and Planner",
            "Planning time (s)",
            FIG_DIR / "simple_planning_time.png",
            SIMPLE_PLANNER_ORDER,
        )
        fig_success_rate_bar(
            simple_df,
            FIG_DIR / "simple_success_rate.png",
            SIMPLE_PLANNER_ORDER,
        )

    if "pnp_phase_success_rate" in all_tables:
        fig_pnp_phase_heatmap(
            all_tables["pnp_phase_success_rate"],
            FIG_DIR / "pnp_phase_heatmap.png",
        )

    print(f"\nDone. Results written to: {OUT_DIR}")


if __name__ == "__main__":
    main()
