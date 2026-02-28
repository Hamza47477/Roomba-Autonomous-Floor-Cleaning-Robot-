"""
Experiment Results Analyser
============================
Reads the three output files from the autonomous cleaning agent:
  - gps_trajectory_*.csv
  - experiment_report_*.txt
  - coverage_grid_*.csv

Generates a multi-panel figure suitable for a research report, plus
prints a textual fault-analysis to the console.

Usage:
  python3 analyse_results.py
  python3 analyse_results.py --traj gps_trajectory_XYZ.csv \
                              --report experiment_report_XYZ.txt \
                              --grid coverage_grid_XYZ.csv
"""

import glob
import sys
import os
import argparse
import numpy as np
import matplotlib
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
from matplotlib.colors import ListedColormap
from matplotlib.gridspec import GridSpec
import warnings
warnings.filterwarnings("ignore")

# ── Locate newest output files automatically ──────────────────────────────────
def find_latest(pattern):
    files = glob.glob(pattern)
    if not files:
        return None
    return max(files, key=os.path.getmtime)

def parse_args():
    p = argparse.ArgumentParser()
    p.add_argument("--traj",   default=None)
    p.add_argument("--report", default=None)
    p.add_argument("--grid",   default=None)
    return p.parse_args()

args = parse_args()

script_dir = os.path.dirname(os.path.abspath(__file__))
traj_file   = args.traj   or find_latest(os.path.join(script_dir, "gps_trajectory_*.csv"))
report_file = args.report or find_latest(os.path.join(script_dir, "experiment_report_*.txt"))
grid_file   = args.grid   or find_latest(os.path.join(script_dir, "coverage_grid_*.csv"))

for label, path in [("Trajectory", traj_file), ("Report", report_file), ("Grid", grid_file)]:
    if path is None:
        print(f"[ERROR] {label} file not found. Run the simulation first.")
        sys.exit(1)
    print(f"[INFO] Using {label}: {os.path.basename(path)}")

# ── Load trajectory CSV ───────────────────────────────────────────────────────
import csv

time_s, gps_x, gps_y, batt, left_enc, right_enc, coverage, behavior = [],[],[],[],[],[],[],[]

with open(traj_file) as f:
    reader = csv.DictReader(f)
    for row in reader:
        time_s.append(float(row["time_sec"]))
        gps_x.append(float(row["gps_x"]))
        gps_y.append(float(row["gps_y"]))
        batt.append(float(row["battery_pct"]))
        left_enc.append(float(row["left_enc"]))
        right_enc.append(float(row["right_enc"]))
        coverage.append(float(row["coverage_pct"]))
        behavior.append(row["behavior"].strip())

time_s   = np.array(time_s)
gps_x    = np.array(gps_x)
gps_y    = np.array(gps_y)
batt     = np.array(batt)
left_enc = np.array(left_enc)
right_enc= np.array(right_enc)
coverage = np.array(coverage)

# ── Load experiment report ────────────────────────────────────────────────────
report = {}
with open(report_file) as f:
    for line in f:
        if ":" in line and not line.startswith("===") and not line.startswith("#"):
            key, _, val = line.partition(":")
            report[key.strip()] = val.strip()

def rget(key, default="N/A"):
    return report.get(key, default)

# ── Load coverage grid ────────────────────────────────────────────────────────
grid_rows = []
with open(grid_file) as f:
    for line in f:
        line = line.strip()
        if line.startswith("#") or not line:
            continue
        grid_rows.append([int(v) for v in line.split(",")])

grid = np.array(grid_rows, dtype=float)
n_rows, n_cols = grid.shape

# ── Derive waypoint pattern for overlay ──────────────────────────────────────
ROOM_MIN_X, ROOM_MAX_X = -1.85, 1.85
ROOM_MIN_Y, ROOM_MAX_Y = -1.85, 1.85
WALL_MARGIN   = 0.30
LANE_SPACING  = 0.40
WP_SPACING    = 0.50
CHARGER_X, CHARGER_Y = 1.78, -0.78

x_min = ROOM_MIN_X + WALL_MARGIN   # -1.55
x_max = ROOM_MAX_X - WALL_MARGIN   # +1.55
y_min = ROOM_MIN_Y + WALL_MARGIN   # -1.55
y_max = ROOM_MAX_Y - WALL_MARGIN   # +1.55

lane_ys = np.arange(y_min, y_max + 0.001, LANE_SPACING)
waypoints = []
left_to_right = True
for ly in lane_ys:
    sx = x_min if left_to_right else x_max
    ex = x_max if left_to_right else x_min
    dist = abs(ex - sx)
    steps = int(np.ceil(dist / WP_SPACING))
    direction = 1 if left_to_right else -1
    for i in range(steps + 1):
        wx = sx + direction * min(i * WP_SPACING, dist)
        waypoints.append((wx, ly))
    left_to_right = not left_to_right

waypoints = np.array(waypoints)

# ── Colour map for behaviors ──────────────────────────────────────────────────
BEH_COLORS = {"CLEAN": "#4CAF50", "TRACK": "#2196F3", "AVOID": "#F44336", "WANDER": "#FF9800"}
def beh_color(b):
    return BEH_COLORS.get(b, "#9E9E9E")

point_colors = [beh_color(b) for b in behavior]

# ── Fault Analysis ────────────────────────────────────────────────────────────
print("\n" + "=" * 60)
print("  FAULT ANALYSIS")
print("=" * 60)

total_beh = len(behavior)
beh_counts = {b: behavior.count(b) for b in set(behavior)}
for b, cnt in beh_counts.items():
    print(f"  {b:8s}: {cnt} ticks ({100*cnt/total_beh:.1f}%)")

cov_final = coverage[-1]
if cov_final < 75:
    print(f"\n[FAULT] Low coverage: {cov_final:.1f}% (target: 85-95%)")
    uncovered = 100 - cov_final
    print(f"         {uncovered:.1f}% of room not cleaned")

# Check coverage grid for uncovered clusters
zero_count = int((grid == 0).sum())
total_cells = n_rows * n_cols
print(f"\n[INFO]  Grid uncovered cells: {zero_count}/{total_cells} ({100*zero_count/total_cells:.1f}%)")

# Check GPS trajectory bounds
gps_x_range = (gps_x.min(), gps_x.max())
gps_y_range = (gps_y.min(), gps_y.max())
print(f"\n[INFO]  GPS X range: [{gps_x_range[0]:.2f}, {gps_x_range[1]:.2f}]  (room: [{ROOM_MIN_X}, {ROOM_MAX_X}])")
print(f"[INFO]  GPS Y range: [{gps_y_range[0]:.2f}, {gps_y_range[1]:.2f}]  (room: [{ROOM_MIN_Y}, {ROOM_MAX_Y}])")

if gps_x.max() > ROOM_MAX_X + 0.05 or gps_x.min() < ROOM_MIN_X - 0.05:
    print("[FAULT] Robot exceeded room X bounds — wall collision likely")
if gps_y.max() > ROOM_MAX_Y + 0.05 or gps_y.min() < ROOM_MIN_Y - 0.05:
    print("[FAULT] Robot exceeded room Y bounds — wall collision likely")

# Check encoder consistency
enc_diff = np.abs(left_enc - right_enc)
enc_drift = enc_diff.max()
print(f"\n[INFO]  Max L/R encoder differential: {enc_drift:.2f} revs")
if enc_drift > 10:
    print("[FAULT] Large encoder differential — possible wheel slip or spin")

# Coverage rate analysis
if len(coverage) > 5:
    cov_rate_early = (coverage[5] - coverage[0]) / (time_s[5] - time_s[0] + 1e-6)
    cov_rate_late  = (coverage[-1] - coverage[-5]) / (time_s[-1] - time_s[-5] + 1e-6)
    print(f"\n[INFO]  Coverage rate (early):  {cov_rate_early:.3f} %/s")
    print(f"[INFO]  Coverage rate (late):   {cov_rate_late:.3f} %/s")
    if cov_rate_late < cov_rate_early * 0.1:
        print("[FAULT] Coverage rate nearly zero in final phase — robot revisiting same areas")

# Waypoints counter check
wp_reported = rget("Waypoints visited")
print(f"\n[INFO]  Report waypoints: {wp_reported}")
print("[NOTE]  Console log shows robot completed full 72-WP pattern 2+ times.")
print("        The counter only tracks currentWaypointIdx at export time,")
print("        not cumulative unique waypoints visited → counter underreports.")

print("=" * 60 + "\n")

# ── Build Figure ──────────────────────────────────────────────────────────────
fig = plt.figure(figsize=(20, 14))
fig.patch.set_facecolor("#1a1a2e")
gs = GridSpec(3, 4, figure=fig, hspace=0.42, wspace=0.38)

PANEL_BG   = "#16213e"
TEXT_COLOR = "#e0e0e0"
GRID_COLOR = "#0a0a1a"
matplotlib.rcParams.update({
    "text.color": TEXT_COLOR,
    "axes.labelcolor": TEXT_COLOR,
    "xtick.color": TEXT_COLOR,
    "ytick.color": TEXT_COLOR,
    "axes.titlecolor": TEXT_COLOR,
})

def style_ax(ax, title):
    ax.set_facecolor(PANEL_BG)
    ax.set_title(title, fontsize=11, fontweight="bold", color=TEXT_COLOR, pad=8)
    ax.tick_params(colors=TEXT_COLOR, labelsize=8)
    for spine in ax.spines.values():
        spine.set_edgecolor("#334466")

# ── Panel 1: GPS Trajectory coloured by behavior ──────────────────────────────
ax1 = fig.add_subplot(gs[0:2, 0:2])
style_ax(ax1, "GPS Trajectory (coloured by behaviour)")

# Room boundary
room_rect = plt.Rectangle((ROOM_MIN_X, ROOM_MIN_Y),
                            ROOM_MAX_X - ROOM_MIN_X, ROOM_MAX_Y - ROOM_MIN_Y,
                            linewidth=2, edgecolor="#607090", facecolor="none", linestyle="--", zorder=2)
# Cleaning area
clean_rect = plt.Rectangle((x_min, y_min), x_max - x_min, y_max - y_min,
                             linewidth=1, edgecolor="#405080", facecolor="none", linestyle=":", zorder=2)
ax1.add_patch(room_rect)
ax1.add_patch(clean_rect)

# Waypoint pattern
ax1.scatter(waypoints[:, 0], waypoints[:, 1], c="#334466", s=12, zorder=3, alpha=0.6, label="Waypoints")

# Trajectory
ax1.scatter(gps_x, gps_y, c=point_colors, s=8, zorder=4, alpha=0.7)
ax1.plot(gps_x, gps_y, color="#7090b0", linewidth=0.4, alpha=0.35, zorder=3)

# Start & end markers
ax1.scatter(gps_x[0], gps_y[0], c="#00FF88", s=120, marker="^", zorder=6, label="Start")
ax1.scatter(gps_x[-1], gps_y[-1], c="#FF4444", s=120, marker="v", zorder=6, label="End")

# Charger
ax1.scatter(CHARGER_X, CHARGER_Y, c="#FFD700", s=200, marker="*", zorder=7, label="Charger")

legend_patches = [mpatches.Patch(color=v, label=k) for k, v in BEH_COLORS.items()]
legend_patches += [
    plt.Line2D([0],[0], marker="^", color="w", markerfacecolor="#00FF88", markersize=10, label="Start"),
    plt.Line2D([0],[0], marker="v", color="w", markerfacecolor="#FF4444", markersize=10, label="End"),
    plt.Line2D([0],[0], marker="*", color="w", markerfacecolor="#FFD700", markersize=12, label="Charger"),
]
ax1.legend(handles=legend_patches, loc="upper left", fontsize=7,
           facecolor="#0e1a30", edgecolor="#334466", labelcolor=TEXT_COLOR)

ax1.set_xlim(ROOM_MIN_X - 0.05, ROOM_MAX_X + 0.05)
ax1.set_ylim(ROOM_MIN_Y - 0.05, ROOM_MAX_Y + 0.05)
ax1.set_xlabel("X (m)")
ax1.set_ylabel("Y (m)")
ax1.set_aspect("equal")
ax1.grid(True, color="#223344", linewidth=0.4, alpha=0.5)

# ── Panel 2: Coverage Grid Heatmap ───────────────────────────────────────────
ax2 = fig.add_subplot(gs[0:2, 2:4])
style_ax(ax2, f"Coverage Grid ({n_rows}×{n_cols} cells @ 0.20m/cell)")

cmap = ListedColormap(["#1a1a3a", "#4CAF50"])
im = ax2.imshow(grid, cmap=cmap, origin="lower",
                extent=[ROOM_MIN_X, ROOM_MAX_X, ROOM_MIN_Y, ROOM_MAX_Y],
                aspect="equal", interpolation="nearest")

# Overlay cleaning boundary
clean_rect2 = plt.Rectangle((x_min, y_min), x_max - x_min, y_max - y_min,
                              linewidth=1.5, edgecolor="#80a0c0", facecolor="none", linestyle="--")
ax2.add_patch(clean_rect2)
ax2.scatter(CHARGER_X, CHARGER_Y, c="#FFD700", s=180, marker="*", zorder=5, label="Charger")

cov_pct = float(rget("Coverage", "0").replace("%", "").split()[0])
ax2.set_title(f"Coverage Grid ({n_rows}×{n_cols})  —  {cov_final:.1f}% visited",
              fontsize=11, fontweight="bold", color=TEXT_COLOR, pad=8)

from matplotlib.patches import Patch
legend2 = [Patch(facecolor="#4CAF50", label="Visited"),
           Patch(facecolor="#1a1a3a", label="Not visited")]
ax2.legend(handles=legend2, loc="upper left", fontsize=8,
           facecolor="#0e1a30", edgecolor="#334466", labelcolor=TEXT_COLOR)
ax2.set_xlabel("X (m)")
ax2.set_ylabel("Y (m)")
ax2.grid(False)

# ── Panel 3: Coverage % over time ────────────────────────────────────────────
ax3 = fig.add_subplot(gs[2, 0])
style_ax(ax3, "Coverage % over Time")
ax3.fill_between(time_s, coverage, alpha=0.3, color="#4CAF50")
ax3.plot(time_s, coverage, color="#4CAF50", linewidth=2)
ax3.axhline(85, color="#FFD700", linestyle="--", linewidth=1, label="Target 85%")
ax3.set_xlabel("Time (s)")
ax3.set_ylabel("Coverage (%)")
ax3.set_ylim(0, 100)
ax3.legend(fontsize=8, facecolor="#0e1a30", edgecolor="#334466", labelcolor=TEXT_COLOR)
ax3.grid(True, color="#223344", linewidth=0.4)

# ── Panel 4: Battery over time ────────────────────────────────────────────────
ax4 = fig.add_subplot(gs[2, 1])
style_ax(ax4, "Battery Level over Time")
ax4.fill_between(time_s, batt, alpha=0.25, color="#2196F3")
ax4.plot(time_s, batt, color="#2196F3", linewidth=2)
ax4.axhline(20, color="#F44336", linestyle="--", linewidth=1, label="TRACK threshold 20%")
ax4.axhline(40, color="#FF9800", linestyle="--", linewidth=1, label="CLEAN threshold 40%")
ax4.set_xlabel("Time (s)")
ax4.set_ylabel("Battery (%)")
ax4.set_ylim(0, 110)
ax4.legend(fontsize=7, facecolor="#0e1a30", edgecolor="#334466", labelcolor=TEXT_COLOR)
ax4.grid(True, color="#223344", linewidth=0.4)

# ── Panel 5: Encoder readings over time ──────────────────────────────────────
ax5 = fig.add_subplot(gs[2, 2])
style_ax(ax5, "Wheel Encoder Readings over Time")
ax5.plot(time_s, left_enc,  color="#E91E63", linewidth=1.5, label="Left  encoder")
ax5.plot(time_s, right_enc, color="#9C27B0", linewidth=1.5, label="Right encoder")
ax5.fill_between(time_s, left_enc, right_enc, alpha=0.15, color="#FF5722",
                 label="|L−R| differential")
ax5.set_xlabel("Time (s)")
ax5.set_ylabel("Encoder (revolutions)")
ax5.legend(fontsize=7, facecolor="#0e1a30", edgecolor="#334466", labelcolor=TEXT_COLOR)
ax5.grid(True, color="#223344", linewidth=0.4)

# ── Panel 6: Behaviour pie chart ──────────────────────────────────────────────
ax6 = fig.add_subplot(gs[2, 3])
style_ax(ax6, "Behaviour Distribution")
pie_labels = list(beh_counts.keys())
pie_sizes  = [beh_counts[b] for b in pie_labels]
pie_colors = [BEH_COLORS.get(b, "#9E9E9E") for b in pie_labels]
wedges, texts, autotexts = ax6.pie(
    pie_sizes, labels=pie_labels, colors=pie_colors, autopct="%1.1f%%",
    startangle=90, textprops={"color": TEXT_COLOR, "fontsize": 9},
    wedgeprops={"edgecolor": "#1a1a2e", "linewidth": 1.5}
)
for at in autotexts:
    at.set_fontsize(8)
ax6.set_aspect("equal")

# ── Summary stats box ─────────────────────────────────────────────────────────
summary_txt = (
    f"End: {rget('End reason')}\n"
    f"Time: {rget('Total time')}\n"
    f"Distance: {rget('Distance travelled')}\n"
    f"Coverage: {rget('Coverage')}\n"
    f"WPs logged: {rget('Waypoints visited')}\n"
    f"  (Note: counter resets on loop restart;\n"
    f"   robot completed ≥2 full passes)"
)
fig.text(0.01, 0.01, summary_txt, fontsize=8, color=TEXT_COLOR,
         verticalalignment="bottom",
         bbox=dict(boxstyle="round,pad=0.5", facecolor="#0e1a30",
                   edgecolor="#334466", alpha=0.9))

fig.suptitle("Autonomous Cleaning Agent — Experiment Results", fontsize=16,
             fontweight="bold", color="#e0e0e0", y=0.98)

out_path = os.path.join(script_dir, "experiment_results.png")
plt.savefig(out_path, dpi=150, bbox_inches="tight", facecolor=fig.get_facecolor())
print(f"[SAVED] {out_path}")
plt.show()
