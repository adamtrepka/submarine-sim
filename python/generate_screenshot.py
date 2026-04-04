"""
Generate a headless screenshot of the submarine GUI charts.

Produces assets/gui.png with the same visual style as gui.py,
but using the non-interactive Agg backend (no display required).

Usage (from repo root):
    python python/generate_screenshot.py
"""

import os

import matplotlib
matplotlib.use("Agg")

import matplotlib.pyplot as plt

from main import SubmarineSim, DEFAULT_HULL_LENGTH_MM, DEFAULT_HULL_DIAMETER_MM

# ---------------------------------------------------------------------------
# Simulation parameters
# ---------------------------------------------------------------------------

SIM_TIME = 30.0          # seconds — enough to see settling
TARGET_DEPTH = 0.5       # metres
DT = 0.001               # simulation timestep

OUTPUT_PATH = os.path.join(
    os.path.dirname(__file__), os.pardir, "assets", "gui.png"
)

# ---------------------------------------------------------------------------
# Run simulation and collect data
# ---------------------------------------------------------------------------

sim = SubmarineSim(target_depth=TARGET_DEPTH)
steps = int(SIM_TIME / DT)

t_data: list[float] = []
depth_true: list[float] = []
depth_meas: list[float] = []
pbs_data: list[float] = []

RECORD_EVERY = 10  # record every 10th step to keep arrays manageable

for i in range(steps):
    sim.step()
    if i % RECORD_EVERY == 0:
        t_data.append(sim.time)
        depth_true.append(sim.depth)
        depth_meas.append(sim.measured_depth)
        pbs_data.append(sim.pbs_ml)

# ---------------------------------------------------------------------------
# Plot (matching gui.py dark theme)
# ---------------------------------------------------------------------------

_GRID_COLOR = "#3b3b54"
_TEXT_COLOR = "#cdd6f4"
_BG = "#1e1e2e"
_PANEL = "#181825"

fig, (ax_depth, ax_pbs) = plt.subplots(
    2, 1,
    figsize=(12, 7),
    height_ratios=[3, 2],
    sharex=True,
)
fig.patch.set_facecolor(_BG)
fig.subplots_adjust(left=0.08, right=0.96, top=0.94, bottom=0.08, hspace=0.25)

for ax in (ax_depth, ax_pbs):
    ax.set_facecolor(_PANEL)
    ax.tick_params(colors=_TEXT_COLOR, which="both")
    ax.xaxis.label.set_color(_TEXT_COLOR)
    ax.yaxis.label.set_color(_TEXT_COLOR)
    ax.title.set_color(_TEXT_COLOR)
    ax.grid(True, color=_GRID_COLOR, linewidth=0.5, alpha=0.6)
    for spine in ax.spines.values():
        spine.set_color(_GRID_COLOR)

# --- Depth chart ---
ax_depth.plot(t_data, depth_true, color="#89b4fa", linewidth=1.2, label="True depth")
ax_depth.plot(t_data, depth_meas, color="#fab387", linewidth=0.8, alpha=0.7, label="Measured")
ax_depth.axhline(TARGET_DEPTH, color="#f38ba8", linewidth=1.0, linestyle="--", label="Target")
ax_depth.set_ylabel("Depth (m)")
ax_depth.set_title("Depth vs Time")
ax_depth.legend(
    loc="upper right", fontsize=8,
    facecolor=_PANEL, edgecolor=_GRID_COLOR, labelcolor=_TEXT_COLOR,
)
ax_depth.invert_yaxis()

# --- PBS chart ---
ax_pbs.plot(t_data, pbs_data, color="#a6e3a1", linewidth=1.2, label="PBS fill")
ax_pbs.axhline(sim.neutral_pbs_ml, color="#6c7086", linewidth=0.8, linestyle="--", label="Neutral")
ax_pbs.set_ylabel("PBS (ml)")
ax_pbs.set_xlabel("Time (s)")
ax_pbs.set_title("PBS Fill vs Time")
ax_pbs.legend(
    loc="upper right", fontsize=8,
    facecolor=_PANEL, edgecolor=_GRID_COLOR, labelcolor=_TEXT_COLOR,
)

# --- Status annotation ---
ax_depth.text(
    0.01, 0.02,
    f"target={TARGET_DEPTH * 1000:.0f}mm  "
    f"final depth={sim.depth * 1000:.1f}mm  "
    f"PBS={sim.pbs_ml:.1f}ml  "
    f"hull={DEFAULT_HULL_LENGTH_MM:.0f}x{DEFAULT_HULL_DIAMETER_MM:.0f}mm",
    transform=ax_depth.transAxes,
    fontsize=8, color=_TEXT_COLOR, verticalalignment="bottom",
    fontfamily="monospace",
)

# --- Save ---
os.makedirs(os.path.dirname(OUTPUT_PATH), exist_ok=True)
fig.savefig(OUTPUT_PATH, dpi=150, facecolor=fig.get_facecolor())
plt.close(fig)

print(f"Screenshot saved to {os.path.abspath(OUTPUT_PATH)}")
