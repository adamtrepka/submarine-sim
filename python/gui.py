"""
Submarine PID Depth Control — Interactive GUI

Real-time animated visualization with:
  - Depth chart (true + measured + target)
  - PBS fill chart
  - Slider for target depth adjustment

Run:  python gui.py
"""

from collections import deque

import matplotlib
matplotlib.use("Qt5Agg")

import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from matplotlib.widgets import Slider

from main import SubmarineSim

# ---------------------------------------------------------------------------
# Configuration
# ---------------------------------------------------------------------------

WINDOW_SEC = 30.0          # visible time window on the x-axis (seconds)
SIM_SPEED = 10             # simulation multiplier (10x real-time)
STEPS_PER_FRAME = None     # auto-calculated below
FPS = 30                   # display frames per second
DEPTH_SLIDER_MIN = 0.0     # metres
DEPTH_SLIDER_MAX = 2.0     # metres
DEPTH_SLIDER_INIT = 0.5    # metres

# How many sim steps per animation frame to run at SIM_SPEED x real-time:
# Each frame covers (1/FPS) real seconds, at SIM_SPEED x that is
# SIM_SPEED / FPS seconds of sim time, divided by dt.
_DT = 0.001
STEPS_PER_FRAME = int(SIM_SPEED / (FPS * _DT))

# Rolling buffer length (samples kept for plotting)
_BUF_LEN = int(WINDOW_SEC / (_DT * STEPS_PER_FRAME)) + 200  # generous headroom


# ---------------------------------------------------------------------------
# Simulation wrapper
# ---------------------------------------------------------------------------

class SimRunner:
    """Thin wrapper that drives SubmarineSim and stores history."""

    def __init__(self, target_depth: float = DEPTH_SLIDER_INIT) -> None:
        self.sim = SubmarineSim(target_depth=target_depth)

        # Ring buffers for plotting
        self.t_buf: deque[float] = deque(maxlen=_BUF_LEN)
        self.depth_true: deque[float] = deque(maxlen=_BUF_LEN)
        self.depth_meas: deque[float] = deque(maxlen=_BUF_LEN)
        self.pbs_buf: deque[float] = deque(maxlen=_BUF_LEN)

        # Snapshot initial state
        self._record()

    def advance(self, n_steps: int) -> None:
        for _ in range(n_steps):
            self.sim.step()
        self._record()

    def _record(self) -> None:
        self.t_buf.append(self.sim.time)
        self.depth_true.append(self.sim.depth)
        self.depth_meas.append(self.sim.measured_depth)
        self.pbs_buf.append(self.sim.pbs_ml)


# ---------------------------------------------------------------------------
# GUI
# ---------------------------------------------------------------------------

def build_gui() -> None:
    runner = SimRunner()

    # --- Figure layout ---
    fig = plt.figure("Submarine PID Depth Control", figsize=(12, 7))
    fig.patch.set_facecolor("#1e1e2e")

    # GridSpec: 2 chart rows + 1 slider row
    gs = fig.add_gridspec(
        3, 1,
        height_ratios=[3, 2, 0.4],
        hspace=0.35,
        left=0.08, right=0.96,
        top=0.94, bottom=0.06,
    )

    ax_depth = fig.add_subplot(gs[0])
    ax_pbs = fig.add_subplot(gs[1], sharex=ax_depth)
    ax_slider = fig.add_subplot(gs[2])

    # --- Styling helper ---
    _GRID_COLOR = "#3b3b54"
    _TEXT_COLOR = "#cdd6f4"
    _BG = "#1e1e2e"
    _PANEL = "#181825"

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
    (ln_true,) = ax_depth.plot([], [], color="#89b4fa", linewidth=1.2, label="True depth")
    (ln_meas,) = ax_depth.plot([], [], color="#fab387", linewidth=0.8, alpha=0.7, label="Measured")
    (ln_target,) = ax_depth.plot([], [], color="#f38ba8", linewidth=1.0, linestyle="--", label="Target")

    ax_depth.set_ylabel("Depth (m)")
    ax_depth.set_title("Depth vs Time")
    ax_depth.legend(loc="upper right", fontsize=8, facecolor=_PANEL, edgecolor=_GRID_COLOR, labelcolor=_TEXT_COLOR)
    ax_depth.set_ylim(-0.05, DEPTH_SLIDER_MAX + 0.1)
    ax_depth.invert_yaxis()  # deeper = higher y visually makes more sense

    # --- PBS chart ---
    (ln_pbs,) = ax_pbs.plot([], [], color="#a6e3a1", linewidth=1.2, label="PBS fill")
    (ln_neutral,) = ax_pbs.plot([], [], color="#6c7086", linewidth=0.8, linestyle="--", label="Neutral")

    ax_pbs.set_ylabel("PBS (ml)")
    ax_pbs.set_xlabel("Time (s)")
    ax_pbs.set_title("PBS Fill vs Time")
    ax_pbs.legend(loc="upper right", fontsize=8, facecolor=_PANEL, edgecolor=_GRID_COLOR, labelcolor=_TEXT_COLOR)
    ax_pbs.set_ylim(0, 30)

    # --- Slider ---
    ax_slider.set_facecolor(_PANEL)
    slider = Slider(
        ax_slider,
        "Target\ndepth (m)",
        DEPTH_SLIDER_MIN,
        DEPTH_SLIDER_MAX,
        valinit=DEPTH_SLIDER_INIT,
        valstep=0.01,
        color="#89b4fa",
    )
    slider.label.set_color(_TEXT_COLOR)
    slider.valtext.set_color(_TEXT_COLOR)

    # --- Status text ---
    status_text = ax_depth.text(
        0.01, 0.02, "", transform=ax_depth.transAxes,
        fontsize=8, color=_TEXT_COLOR, verticalalignment="bottom",
        fontfamily="monospace",
    )

    # --- Settle-time tracking ---
    _TOLERANCE_M = 0.010  # ±10 mm

    class _SettleState:
        """Mutable state for settle-time measurement."""
        t_change: float = 0.0        # sim time of last slider change
        settled: bool = False         # True once depth entered tolerance band
        settle_time: float | None = None  # seconds from change to settled

    _settle = _SettleState()

    settle_text = ax_depth.text(
        0.99, 0.02, "", transform=ax_depth.transAxes,
        fontsize=9, color="#a6e3a1", verticalalignment="bottom",
        horizontalalignment="right", fontfamily="monospace",
        fontweight="bold",
    )

    # --- Slider callback ---
    def on_slider_change(val: float) -> None:
        runner.sim.target_depth = val
        _settle.t_change = runner.sim.time
        _settle.settled = False
        _settle.settle_time = None

    slider.on_changed(on_slider_change)

    # --- Animation update ---
    def update(_frame: int):
        runner.advance(STEPS_PER_FRAME)

        t = list(runner.t_buf)
        if len(t) < 2:
            return ln_true, ln_meas, ln_target, ln_pbs, ln_neutral, status_text, settle_text

        t_min = max(0.0, t[-1] - WINDOW_SEC)
        t_max = t[-1] + 1.0

        # Depth lines
        ln_true.set_data(t, list(runner.depth_true))
        ln_meas.set_data(t, list(runner.depth_meas))
        ln_target.set_data([t_min, t_max], [runner.sim.target_depth] * 2)

        ax_depth.set_xlim(t_min, t_max)

        # Auto-scale depth axis around visible data
        visible_depths = [
            d for tt, d in zip(t, runner.depth_true) if tt >= t_min
        ]
        if visible_depths:
            d_lo = min(min(visible_depths), runner.sim.target_depth) - 0.05
            d_hi = max(max(visible_depths), runner.sim.target_depth) + 0.05
            ax_depth.set_ylim(d_hi, d_lo)  # inverted

        # PBS lines
        ln_pbs.set_data(t, list(runner.pbs_buf))
        ln_neutral.set_data([t_min, t_max], [runner.sim.neutral_pbs_ml] * 2)

        ax_pbs.set_xlim(t_min, t_max)

        # Auto-scale PBS axis
        visible_pbs = [p for tt, p in zip(t, runner.pbs_buf) if tt >= t_min]
        if visible_pbs:
            p_lo = max(0, min(visible_pbs) - 1.0)
            p_hi = min(30, max(visible_pbs) + 1.0)
            ax_pbs.set_ylim(p_lo, p_hi)

        # Status text
        status_text.set_text(
            f"t={runner.sim.time:.1f}s  "
            f"depth={runner.sim.depth * 1000:.1f}mm  "
            f"target={runner.sim.target_depth * 1000:.0f}mm  "
            f"PBS={runner.sim.pbs_ml:.1f}ml  "
            f"v={runner.sim.velocity * 1000:.1f}mm/s"
        )

        # Settle-time detection
        depth_error = abs(runner.sim.depth - runner.sim.target_depth)
        if not _settle.settled and depth_error <= _TOLERANCE_M:
            _settle.settled = True
            _settle.settle_time = runner.sim.time - _settle.t_change

        if _settle.settle_time is not None:
            settle_text.set_text(f"Settle: {_settle.settle_time:.1f}s")
            settle_text.set_color("#a6e3a1")  # green
        elif _settle.t_change > 0:
            elapsed = runner.sim.time - _settle.t_change
            settle_text.set_text(f"Settling... {elapsed:.1f}s")
            settle_text.set_color("#f9e2af")  # yellow
        else:
            settle_text.set_text("")

        return ln_true, ln_meas, ln_target, ln_pbs, ln_neutral, status_text, settle_text

    _anim = FuncAnimation(  # noqa: F841  prevent GC
        fig,
        update,
        interval=int(1000 / FPS),
        blit=False,
        cache_frame_data=False,
    )

    plt.show()


if __name__ == "__main__":
    build_gui()
