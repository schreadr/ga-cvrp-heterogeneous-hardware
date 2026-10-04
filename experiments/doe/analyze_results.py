import json
import os
import re
from pathlib import Path

# Set writable cache dir for matplotlib
os.environ.setdefault("MPLCONFIGDIR", "/tmp/matplotlib")

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
import seaborn as sns
import statsmodels.api as sm
from statsmodels.formula.api import ols


RESULTS_DIR = Path("results")

BKS = {
    "P-n70-k10": 827,
    "P-n76-k4": 593,
}

records = []
histories = []

# 1. Parse all experiment JSONs and histories
for f in RESULTS_DIR.glob("*.json"):
    with open(f) as fp:
        data = json.load(fp)
    
    match = re.search(r"(?P<inst>[^_]+)_p(?P<p>\d+)_g(?P<g>\d+)_e(?P<e>[\d\.]+)_k(?P<k>\d+)_r(?P<r>\d+)_c(?P<c>\d+)_m(?P<m>\d+)_s(?P<s>\d+)", f.name)
    if not match:
        continue
    cfg = {k: int(v) if k not in ("inst", "e") else (float(v) if k == "e" else v) for k, v in match.groupdict().items()}
    
    inst = cfg["inst"]
    bks_val = BKS.get(inst, np.nan)
    exact_fitness = data["solution"]["exact_best_fitness"]
    gap_pct = (exact_fitness - bks_val) / bks_val * 100.0 if not np.isnan(bks_val) else np.nan
    
    # Calculate Time-to-Target (5% from BKS)
    target_5pct = bks_val * 1.05 if not np.isnan(bks_val) else None
    ttt_5pct_ms = np.nan
    
    history = data.get("best_fitness_history", [])
    for rec in history:
        if target_5pct and rec["best_fitness"] <= target_5pct:
            ttt_5pct_ms = rec["timestamp_ms"]
            break
            
    records.append({
        **cfg,
        "wall_time_s": data["timings"]["total_wall_time_ms"] / 1000.0,
        "exact_fitness": exact_fitness,
        "gap_pct": gap_pct,
        "ttt_5pct_s": ttt_5pct_ms / 1000.0 if not np.isnan(ttt_5pct_ms) else np.nan,
        "throughput": data["timings"]["throughput_evals_sec"],
        "fpga_util": data["timings"]["workers"][0]["utilization_pct"] if data["timings"].get("workers") else np.nan
    })

df = pd.DataFrame(records)

# ---------------------------------------------------------
# A. Multi-Factor ANOVA: Main Effects & Interactions
# ---------------------------------------------------------
print("=" * 85)
print("             MULTI-FACTOR ANOVA (SOLUTION QUALITY: GAP TO BKS %)")
print("=" * 85)
formula_qual = (
    "gap_pct ~ C(inst) + C(p) + C(c) + C(m) + C(k) + C(r) "
    "+ C(inst):C(m) + C(inst):C(p) + C(p):C(m) + C(k):C(r)"
)
model_qual = ols(formula_qual, data=df).fit()
anova_qual = sm.stats.anova_lm(model_qual, typ=2)
anova_qual["eta_sq (%)"] = (anova_qual["sum_sq"] / anova_qual["sum_sq"].sum()) * 100.0

display_cols = ["sum_sq", "df", "F", "PR(>F)", "eta_sq (%)"]
print(anova_qual[display_cols].to_string(formatters={
    "sum_sq": "{:10.2f}".format,
    "F": "{:8.2f}".format,
    "PR(>F)": lambda x: f"{x:8.2e}" if pd.notnull(x) else "     NaN",
    "eta_sq (%)": "{:6.2f}%".format,
}))

print("=" * 85 + "\n")

# ---------------------------------------------------------
# B. Pareto Frontier Analysis (Quality vs. Wall-Clock Time)
# ---------------------------------------------------------
cfg_agg = df.groupby(["inst", "p", "c", "m", "k", "r"]).agg(
    mean_time_s=("wall_time_s", "mean"),
    mean_gap=("gap_pct", "mean"),
    min_gap=("gap_pct", "min"),
    runs_under_1_5pct=("gap_pct", lambda s: (s <= 1.5).sum()),
    runs_under_1_0pct=("gap_pct", lambda s: (s <= 1.0).sum()),
    total_runs=("gap_pct", "count")
).reset_index()

pareto_records = []
print("=" * 95)
print("             PARETO FRONTIER: TIME VS. SOLUTION QUALITY (ACROSS SEEDS)")
print("=" * 95)

for inst in ["P-n70-k10", "P-n76-k4"]:
    print(f"\n--- Instance: {inst} ---")
    sub = cfg_agg[cfg_agg["inst"] == inst].sort_values("mean_time_s")
    best_gap = float("inf")
    pareto_inst = []
    for _, row in sub.iterrows():
        if row["mean_gap"] < best_gap:
            best_gap = row["mean_gap"]
            r_dict = row.to_dict()
            pareto_inst.append(r_dict)
            pareto_records.append(r_dict)
            
    inst_pdf = pd.DataFrame(pareto_inst)
    cols = ["p", "c", "m", "k", "r", "mean_time_s", "mean_gap", "min_gap", "runs_under_1_5pct", "runs_under_1_0pct"]
    print(inst_pdf[cols].to_string(index=False, formatters={
        "mean_time_s": "{:8.2f}s".format,
        "mean_gap": "{:6.2f}%".format,
        "min_gap": "{:6.2f}%".format,
        "runs_under_1_5pct": "{:d}/5".format,
        "runs_under_1_0pct": "{:d}/5".format,
    }))

print("=" * 95 + "\n")

pareto_df = pd.DataFrame(pareto_records)

# ---------------------------------------------------------
# C. Export Individual Publication-Ready Plots
# ---------------------------------------------------------
sns.set_theme(style="whitegrid", font_scale=1.05)

# 1. Quality vs. k for each instance separately (standard styling, shaded 95% CI)
for inst in ["P-n70-k10", "P-n76-k4"]:
    fig, ax = plt.subplots(figsize=(6, 4.2))
    inst_df = df[df["inst"] == inst]
    sns.lineplot(
        data=inst_df, x="k", y="gap_pct", hue="r", style="r",
        marker="o", ax=ax
    )
    ax.set_title(f"{inst}: Gap to BKS vs. 2-Opt Iterations (k)")
    ax.set_xlabel("2-Opt Iterations per Subtour (k)")
    ax.set_ylabel("Gap to BKS (%)")
    ax.set_xticks([5, 30, 100])
    plt.tight_layout()
    fig.savefig(f"quality_k_{inst}.png", dpi=200)
    plt.close(fig)
    print(f"Saved: quality_k_{inst}.png")

# 2. Pareto Frontier plots for each instance separately (distinguishing k, p, and r)
for inst in ["P-n70-k10", "P-n76-k4"]:
    fig, ax = plt.subplots(figsize=(7.2, 4.8))
    inst_cfg = cfg_agg[cfg_agg["inst"] == inst].copy()
    inst_cfg["k"] = inst_cfg["k"].map({5: "k=5", 30: "k=30", 100: "k=100"})
    inst_cfg["r"] = inst_cfg["r"].map({0: "r=0", 1: "r=1"})
    inst_cfg["p"] = inst_cfg["p"].map({10: "p=10", 20: "p=20"})
    inst_pareto = pareto_df[pareto_df["inst"] == inst].sort_values("mean_time_s").copy()
    
    # Scatter of all configurations:
    # - Color (hue): k (k=5, k=30, k=100)
    # - Shape (style): r (r=0, r=1)
    # - Size: p (p=10, p=20)
    sns.scatterplot(
        data=inst_cfg, x="mean_time_s", y="mean_gap",
        hue="k", hue_order=["k=5", "k=30", "k=100"],
        style="r", style_order=["r=0", "r=1"],
        size="p", size_order=["p=10", "p=20"],
        markers={"r=0": "o", "r=1": "s"},
        sizes={"p=10": 55, "p=20": 125},
        palette="tab10", alpha=0.75, ax=ax
    )
    # Pareto step-line
    ax.step(
        inst_pareto["mean_time_s"], inst_pareto["mean_gap"],
        where="post", color="tab:red", linestyle="--", linewidth=2.0, label="Pareto Frontier"
    )
    # Highlight Pareto points with matching shape (r) and size (p)
    for _, prow in inst_pareto.iterrows():
        marker = "s" if prow["r"] == 1 else "o"
        size = 135 if prow["p"] == 20 else 70
        ax.scatter(
            prow["mean_time_s"], prow["mean_gap"],
            color="tab:red", marker=marker, edgecolor="black", s=size, zorder=5
        )
        
    ax.set_title(f"{inst}: Wall-Clock Time vs. Gap %")
    ax.set_xlabel("Mean Wall-Clock Time (s)")
    ax.set_ylabel("Mean Gap to BKS (%)")
    
    handles, labels = ax.get_legend_handles_labels()
    filtered = [(h, l) for h, l in zip(handles, labels) if l not in ("k", "r", "p")]
    filt_handles, filt_labels = zip(*filtered) if filtered else ([], [])
    ax.legend(
        filt_handles, filt_labels,
        bbox_to_anchor=(1.02, 1), loc="upper left", borderaxespad=0.,
        fontsize=8.5, title=None
    )
    plt.tight_layout()
    fig.savefig(f"pareto_{inst}.png", dpi=200, bbox_inches="tight")
    plt.close(fig)
    print(f"Saved: pareto_{inst}.png")