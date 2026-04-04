"""
Run submarine simulation for several hull geometry variants and print a summary table.

Usage:
    python python/run_variants.py
"""

from main import SubmarineSim, DEFAULT_HULL_LENGTH_MM, DEFAULT_HULL_DIAMETER_MM

TOLERANCE_M = 0.010  # ±10 mm settle tolerance

VARIANTS = [
    {"length_mm": 250, "diameter_mm": 45},
    {"length_mm": 270, "diameter_mm": 48},
    {"length_mm": 300, "diameter_mm": 50},   # default
    {"length_mm": 320, "diameter_mm": 52},
    {"length_mm": 350, "diameter_mm": 55},
    {"length_mm": 380, "diameter_mm": 58},
]

SIM_TIME = 60.0
TARGET_DEPTH = 0.5


def run_variant(length_mm: float, diameter_mm: float) -> dict:
    sim = SubmarineSim(
        target_depth=TARGET_DEPTH,
        hull_length_mm=length_mm,
        hull_diameter_mm=diameter_mm,
    )

    steps = int(SIM_TIME / sim.dt)
    settle_time = None

    for _ in range(steps):
        sim.step()
        if settle_time is None and abs(sim.depth - sim.target_depth) <= TOLERANCE_M:
            settle_time = sim.time

    overshoot = (
        (sim.max_depth - sim.target_depth) * 1000
        if sim.max_depth > sim.target_depth
        else 0.0
    )

    return {
        "length_mm": length_mm,
        "diameter_mm": diameter_mm,
        "mass_g": sim._M * 1000,
        "neutral_pbs": sim.neutral_pbs_ml,
        "final_depth_mm": sim.depth * 1000,
        "final_pbs": sim.pbs_ml,
        "overshoot_mm": overshoot,
        "settle_time": settle_time,
        "max_depth_mm": sim.max_depth * 1000,
    }


def main() -> None:
    results = []
    for v in VARIANTS:
        print(f"Running: length={v['length_mm']}mm, diameter={v['diameter_mm']}mm ...")
        results.append(run_variant(v["length_mm"], v["diameter_mm"]))

    # Print summary table
    print()
    print("=" * 115)
    print(f"  Submarine PID Simulation — Hull Geometry Variants  (target={TARGET_DEPTH}m, time={SIM_TIME}s)")
    print("=" * 115)
    header = (
        f"{'Length(mm)':>10} {'Diam(mm)':>9} {'Mass(g)':>8} {'NeutPBS(ml)':>12} "
        f"{'Final(mm)':>10} {'PBS(ml)':>8} {'Overshoot(mm)':>14} "
        f"{'Settle(s)':>10} {'MaxDepth(mm)':>13}"
    )
    print(header)
    print("-" * 115)

    for r in results:
        settle_str = f"{r['settle_time']:.1f}" if r["settle_time"] is not None else "N/A"
        print(
            f"{r['length_mm']:>10.0f} {r['diameter_mm']:>9.0f} {r['mass_g']:>8.0f} {r['neutral_pbs']:>12.1f} "
            f"{r['final_depth_mm']:>10.1f} {r['final_pbs']:>8.2f} {r['overshoot_mm']:>14.1f} "
            f"{settle_str:>10} {r['max_depth_mm']:>13.1f}"
        )

    print("=" * 115)


if __name__ == "__main__":
    main()
