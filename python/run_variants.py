"""
Run submarine simulation for several weight/volume variants and print a summary table.

Usage:
    python python/run_variants.py
"""

from main import SubmarineSim, DEFAULT_HULL_MASS_G, DEFAULT_VOLUME_ML

TOLERANCE_M = 0.010  # ±10 mm settle tolerance

VARIANTS = [
    {"weight_g": 500, "volume_ml": 520},
    {"weight_g": 550, "volume_ml": 570},
    {"weight_g": 570, "volume_ml": 589},
    {"weight_g": 600, "volume_ml": 620},
    {"weight_g": 650, "volume_ml": 665},
    {"weight_g": 700, "volume_ml": 720},
]

SIM_TIME = 60.0
TARGET_DEPTH = 0.5


def run_variant(weight_g: float, volume_ml: float) -> dict:
    sim = SubmarineSim(
        target_depth=TARGET_DEPTH,
        hull_mass_g=weight_g,
        volume_ml=volume_ml,
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
        "weight_g": weight_g,
        "volume_ml": volume_ml,
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
        print(f"Running: weight={v['weight_g']}g, volume={v['volume_ml']}ml ...")
        results.append(run_variant(v["weight_g"], v["volume_ml"]))

    # Print summary table
    print()
    print("=" * 105)
    print(f"  Submarine PID Simulation — Weight/Volume Variants  (target={TARGET_DEPTH}m, time={SIM_TIME}s)")
    print("=" * 105)
    header = (
        f"{'Weight(g)':>10} {'Volume(ml)':>11} {'NeutPBS(ml)':>12} "
        f"{'Final(mm)':>10} {'PBS(ml)':>8} {'Overshoot(mm)':>14} "
        f"{'Settle(s)':>10} {'MaxDepth(mm)':>13}"
    )
    print(header)
    print("-" * 105)

    for r in results:
        settle_str = f"{r['settle_time']:.1f}" if r["settle_time"] is not None else "N/A"
        print(
            f"{r['weight_g']:>10.0f} {r['volume_ml']:>11.0f} {r['neutral_pbs']:>12.1f} "
            f"{r['final_depth_mm']:>10.1f} {r['final_pbs']:>8.2f} {r['overshoot_mm']:>14.1f} "
            f"{settle_str:>10} {r['max_depth_mm']:>13.1f}"
        )

    print("=" * 105)


if __name__ == "__main__":
    main()
