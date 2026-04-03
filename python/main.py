"""
Submarine PID Depth Control with Realistic Sensor Models

Sensors modelled from datasheets:
  - MS5837-30BA pressure sensor (depth): 0.016 mbar resolution, ~0.16mm depth
  - MPU6050 accelerometer (+-2g):  0.061 mg/LSB, 400 ug/sqrt(Hz) noise density

Velocity estimation: complementary filter fusing accelerometer integration
                     (short-term) with depth-sensor differentiation (long-term)
"""

import math
import random


# --- Gaussian RNG helper (Box-Muller) ---

class GaussianRng:
    """Deterministic Gaussian RNG using Box-Muller transform."""

    def __init__(self, seed: int = 42):
        self._rng = random.Random(seed)
        self._has_spare = False
        self._spare = 0.0

    def next(self, mean: float, std_dev: float) -> float:
        if self._has_spare:
            self._has_spare = False
            return mean + std_dev * self._spare

        while True:
            u = self._rng.random() * 2.0 - 1.0
            v = self._rng.random() * 2.0 - 1.0
            s = u * u + v * v
            if 0.0 < s < 1.0:
                break

        s = math.sqrt(-2.0 * math.log(s) / s)
        self._spare = v * s
        self._has_spare = True
        return mean + std_dev * u * s


# Single shared instance (mirrors C# static class behaviour)
_gaussian_rng = GaussianRng(42)


# --- MS5837-30BA Pressure/Depth Sensor Model ---

class MS5837Sensor:
    """
    Datasheet specs (OSR 8192):
      Resolution: 0.016 mbar  ->  depth step ~ 0.016 / 98.1 m ~ 0.163 mm
      RMS noise:  ~0.016 mbar ->  sigma ~ 0.163 mm depth
      Conversion: ~18 ms per reading (P+T -> ~36 ms -> ~28 Hz; we use 50 Hz)
      Accuracy:   +/-2 mbar   ->  +/-20.4 mm offset (factory bias, calibratable)
    """

    _PA_PER_MBAR_DIV = 98.1        # Pa per mbar for water depth: rho*g/100
    _RESOLUTION_MBAR = 0.016
    _NOISE_SIGMA_MBAR = 0.016
    _ACCURACY_MBAR = 2.0

    def __init__(self) -> None:
        self.sample_period: float = 0.020  # 50 Hz
        self.depth_resolution_m: float = self._RESOLUTION_MBAR / 98.1

        self._noise_sigma_m: float = self._NOISE_SIGMA_MBAR / 98.1
        self._bias_m: float = (
            self._ACCURACY_MBAR * (2.0 * random.Random(123).random() - 1.0)
        ) / 98.1
        self._last_reading: float = 0.0
        self._time_since_last_sample: float = self.sample_period  # force first sample
        self._new_sample: bool = False

    def read(self, true_depth_m: float, dt: float) -> tuple[float, bool]:
        """Returns (reading, is_new_sample). Only updates at sensor sample rate."""
        self._time_since_last_sample += dt
        self._new_sample = False
        if self._time_since_last_sample >= self.sample_period:
            self._time_since_last_sample = 0.0
            self._new_sample = True

            noisy_depth = (
                true_depth_m + self._bias_m + _gaussian_rng.next(0, self._noise_sigma_m)
            )
            self._last_reading = (
                round(noisy_depth / self.depth_resolution_m) * self.depth_resolution_m
            )
        return self._last_reading, self._new_sample

    @property
    def bias_m(self) -> float:
        return self._bias_m


# --- MPU6050 Accelerometer Model (+-2g range, vertical axis only) ---

class MPU6050Accel:
    """
    Datasheet specs (+-2g, DLPF_CFG=6 -> BW=5Hz):
      Sensitivity:   16384 LSB/g
      Resolution:    0.061 mg/LSB -> 0.000598 m/s2
      Noise density: 400 ug/sqrt(Hz), effective BW ~6.5 Hz -> sigma ~ 1.02 mg ~ 0.01 m/s2
      Zero-g bias:   +-50 mg (X/Y), +-80 mg (Z)
      Sample rate:   200 Hz (DLPF enabled, SMPLRT_DIV=4)
    """

    _G_MPS2 = 9.80665
    _SENSITIVITY = 16384.0
    _LSB_MPS2 = _G_MPS2 / _SENSITIVITY   # ~ 0.000598 m/s2

    _NOISE_DENSITY = 400e-6              # g/sqrt(Hz)
    _EFFECTIVE_NOISE_BW = 6.5            # Hz
    _BIAS_MAX_MG = 50.0

    def __init__(self) -> None:
        self.sample_period: float = 0.005  # 200 Hz

        self._noise_sigma_mps2: float = (
            self._NOISE_DENSITY * math.sqrt(self._EFFECTIVE_NOISE_BW) * self._G_MPS2
        )
        self._bias_mps2: float = (
            self._BIAS_MAX_MG * (2.0 * random.Random(456).random() - 1.0)
        ) * 1e-3 * self._G_MPS2
        self._last_reading: float = 0.0
        self._time_since_last_sample: float = self.sample_period

    def read(self, true_accel_mps2: float, dt: float) -> float:
        """
        Returns measured net dynamic acceleration (m/s2, positive = down).
        Gravity is NOT included -- caller provides net dynamic accel only.
        """
        self._time_since_last_sample += dt
        if self._time_since_last_sample >= self.sample_period:
            self._time_since_last_sample = 0.0
            noisy = (
                true_accel_mps2
                + self._bias_mps2
                + _gaussian_rng.next(0, self._noise_sigma_mps2)
            )
            self._last_reading = (
                round(noisy / self._LSB_MPS2) * self._LSB_MPS2
            )
        return self._last_reading

    @property
    def bias_mps2(self) -> float:
        return self._bias_mps2

    @property
    def noise_sigma(self) -> float:
        return self._noise_sigma_mps2


# --- PID Controller (full P + I + D) ---

class PidController:
    def __init__(
        self,
        kp: float,
        ki: float,
        kd: float,
        set_point: float,
        output_min: float = float("-inf"),
        output_max: float = float("inf"),
        dead_band: float = 0.0,
    ) -> None:
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.set_point = set_point
        self.output_min = output_min
        self.output_max = output_max
        self.dead_band = dead_band
        self._integral: float = 0.0

    def update(self, process_value: float, velocity: float, dt: float) -> float:
        """
        PID update. Velocity is provided externally (from sensor fusion)
        for the derivative term -- derivative-on-measurement, not on error.
        """
        error = self.set_point - process_value

        # Dead band on P and I only (not D -- need full velocity signal)
        error_for_pi = 0.0 if abs(error) <= self.dead_band else error

        p_term = self.kp * error_for_pi

        # D term: negative velocity * Kd (derivative on measurement)
        d_term = -self.kd * velocity

        # Anti-windup: only integrate when output is not saturated,
        # or when error would reduce the integral (back-off)
        candidate_integral = self._integral + error_for_pi * dt
        candidate_output = p_term + self.ki * candidate_integral + d_term
        if self.output_min <= candidate_output <= self.output_max:
            self._integral = candidate_integral  # within bounds
        elif error_for_pi * self._integral < 0:
            self._integral = candidate_integral  # error opposes integral -> unwind
        # else: saturated and error would make it worse -> freeze integral

        i_term = self.ki * self._integral

        return max(self.output_min, min(self.output_max, p_term + i_term + d_term))

    def reset(self) -> None:
        self._integral = 0.0


# --- Accelerometer Bias Calibrator ---
# At startup the submarine is nearly stationary (true accel ~ 0).
# Average N samples to estimate the sensor bias, then subtract it.

class AccelCalibrator:
    def __init__(self, calibration_time_sec: float, sample_period: float) -> None:
        self._required_samples: int = max(1, int(calibration_time_sec / sample_period))
        self._sum: float = 0.0
        self._count: int = 0
        self._estimated_bias: float = 0.0
        self._done: bool = False

    def add_sample(self, raw_accel: float) -> bool:
        """Feed raw accel sample. Returns True when calibration is complete."""
        if self._done:
            return True
        self._sum += raw_accel
        self._count += 1
        if self._count >= self._required_samples:
            self._estimated_bias = self._sum / self._count
            self._done = True
        return self._done

    @property
    def estimated_bias(self) -> float:
        return self._estimated_bias

    @property
    def is_done(self) -> bool:
        return self._done


# --- Complementary Filter Velocity Estimator ---

class VelocityFusion:
    """
    Fuses accelerometer integration (short-term, bias-calibrated)
    with depth-sensor differentiation (long-term, drift-free).

    Predict:  v_fused += calibratedAccel * dt
    Correct:  on each depth sample, blend toward depth-derived velocity
              beta = dt_depth / (tau + dt_depth) where dt_depth = depth sample interval
    """

    def __init__(self, tau: float = 0.5) -> None:
        self._tau = tau
        self._fused_velocity: float = 0.0
        self._prev_depth: float = 0.0
        self._depth_age: float = 0.0
        self._first_depth: bool = True

    def update(
        self,
        calibrated_accel: float,
        depth: float,
        new_depth_sample: bool,
        dt: float,
    ) -> float:
        # Predict: integrate calibrated accelerometer
        self._fused_velocity += calibrated_accel * dt
        self._depth_age += dt

        # Correct: when depth sensor updates, blend toward depth-derived velocity
        if new_depth_sample:
            if self._first_depth:
                self._prev_depth = depth
                self._first_depth = False
            elif self._depth_age > 1e-9:
                v_depth = (depth - self._prev_depth) / self._depth_age
                self._prev_depth = depth

                beta = self._depth_age / (self._tau + self._depth_age)
                self._fused_velocity = (
                    (1.0 - beta) * self._fused_velocity + beta * v_depth
                )
            self._depth_age = 0.0

        return self._fused_velocity

    @property
    def velocity(self) -> float:
        return self._fused_velocity


# --- Submarine Simulation ---

G = 9.81
RHO = 1000.0

L = 0.30
R = 0.025
HULL_MASS = 0.570

PBS_MAX_ML = 30.0

CD = 1.0
DRAG_AREA = L * 2 * R


def submerged_volume(d: float) -> float:
    if d >= R:
        return math.pi * R * R * L
    if d <= -R:
        return 0.0
    area = R * R * math.acos(-d / R) + d * math.sqrt(R * R - d * d)
    return area * L


def find_equilibrium(total_mass_kg: float) -> float:
    lo, hi = -R, R
    for _ in range(200):
        mid = (lo + hi) / 2.0
        buoyancy = RHO * submerged_volume(mid)
        if buoyancy < total_mass_kg:
            lo = mid
        else:
            hi = mid
    return (lo + hi) / 2.0


def main() -> None:
    neutral_pbs_ml = (RHO * math.pi * R * R * L - HULL_MASS) * 1e3

    init_pbs_ml = neutral_pbs_ml - 1.0
    init_mass = HULL_MASS + init_pbs_ml * 1e-3
    d0 = find_equilibrium(init_mass)

    # --- Sensors ---
    depth_sensor = MS5837Sensor()
    accel_sensor = MPU6050Accel()

    # Accelerometer bias calibration: 0.5s at startup while stationary
    calib_time_sec = 0.5
    accel_calib = AccelCalibrator(calib_time_sec, accel_sensor.sample_period)

    # Velocity fusion: tau = 0.5s complementary filter
    fusion_tau = 0.5
    vel_fusion = VelocityFusion(tau=fusion_tau)

    max_pump_rate = 1.5

    # --- PID ---
    pid = PidController(
        kp=10.0,
        ki=0.1,
        kd=80.0,
        set_point=0.5,
        output_min=-neutral_pbs_ml,
        output_max=PBS_MAX_ML - neutral_pbs_ml,
        dead_band=0.001,
    )

    dt = 0.001
    sim_time = 60.0
    steps = int(sim_time / dt)
    print_every = int(1.0 / dt)

    depth = d0
    vel = 0.0
    pbs_ml = init_pbs_ml
    max_depth = 0.0

    # Track previous accel reading to detect new samples for calibration
    prev_raw_accel = float("nan")

    # --- Header ---
    print("=== Submarine PID Depth Control ===")
    print(
        f"Cylinder: {L * 100:.0f}cm x dia {R * 200:.0f}cm, "
        f"Hull: {HULL_MASS * 1000:.0f}g"
    )
    print(
        f"Volume: {math.pi * R * R * L * 1e6:.1f}ml, "
        f"Neutral PBS: {neutral_pbs_ml:.1f}ml"
    )
    print(f"Init: PBS={init_pbs_ml:.1f}ml, depth={d0 * 1000:.1f}mm")
    print()
    print("Depth sensor: MS5837-30BA (OSR 8192)")
    print(
        f"  Resolution: {depth_sensor.depth_resolution_m * 1000:.3f}mm, "
        f"Rate: {1.0 / depth_sensor.sample_period:.0f}Hz, "
        f"Bias: {depth_sensor.bias_m * 1000:.2f}mm"
    )
    print("Accel: MPU6050 (+-2g, DLPF 5Hz)")
    print(
        f"  True bias: {accel_sensor.bias_mps2 * 1000:.2f}mm/s2, "
        f"Noise sigma: {accel_sensor.noise_sigma * 1000:.2f}mm/s2"
    )
    print(f"  Calibration: {calib_time_sec:.1f}s at startup")
    print(f"Pump: max {max_pump_rate:.1f} ml/s")
    print("Target: 0.500m")
    print()
    print(
        f"PID: Kp={pid.kp}, Ki={pid.ki}, Kd={pid.kd}, "
        f"DeadBand={pid.dead_band * 1000:.1f}mm"
    )
    print(f"Velocity fusion tau={fusion_tau:.1f}s")
    print()
    print(
        f"{'t(s)':>6} {'True(m)':>9} {'Sens(m)':>9} {'v(mm/s)':>9} "
        f"{'vEst(mm/s)':>11} {'PBS(ml)':>8} {'H2O(g)':>8} {'a(m/s2)':>9}"
    )
    print("-" * 80)

    for i in range(steps + 1):
        # --- Physics ---
        total_mass = HULL_MASS + pbs_ml * 1e-3
        weight = total_mass * G
        buoyancy = RHO * G * submerged_volume(depth)
        drag = 0.5 * RHO * CD * DRAG_AREA * vel * abs(vel)
        accel = (weight - buoyancy - drag) / total_mass

        # --- Sensor readings ---
        measured_depth, new_depth_sample = depth_sensor.read(depth, dt)
        raw_accel = accel_sensor.read(accel, dt)

        # Detect new accelerometer sample (value changed = new conversion)
        new_accel_sample = raw_accel != prev_raw_accel
        prev_raw_accel = raw_accel

        # --- Accelerometer bias calibration ---
        if not accel_calib.is_done:
            if new_accel_sample:
                accel_calib.add_sample(raw_accel)

            # During calibration: no PID, hold PBS constant, no velocity estimation
            vel += accel * dt
            depth += vel * dt
            if depth < -R:
                depth = -R
                if vel < 0:
                    vel = 0.0
            if depth > max_depth:
                max_depth = depth

            if i % print_every == 0:
                print(
                    f"{i * dt:6.1f} {depth:9.4f} {measured_depth:9.4f} "
                    f"{vel * 1000:9.2f} {'[CALIB]':>11} "
                    f"{pbs_ml:8.2f} {pbs_ml:8.2f} {accel:9.5f}"
                )

            if accel_calib.is_done:
                print(
                    f"  >>> Accel calibration done: estimated bias = "
                    f"{accel_calib.estimated_bias * 1000:.2f} mm/s2 "
                    f"(true: {accel_sensor.bias_mps2 * 1000:.2f} mm/s2)"
                )
            continue

        # --- Calibrated accelerometer ---
        calibrated_accel = raw_accel - accel_calib.estimated_bias

        # --- Velocity fusion ---
        est_velocity = vel_fusion.update(
            calibrated_accel, measured_depth, new_depth_sample, dt
        )

        # --- PID ---
        correction = pid.update(measured_depth, est_velocity, dt)
        target_pbs = max(0.0, min(PBS_MAX_ML, neutral_pbs_ml + correction))

        max_change = max_pump_rate * dt
        pbs_ml += max(-max_change, min(max_change, target_pbs - pbs_ml))
        pbs_ml = max(0.0, min(PBS_MAX_ML, pbs_ml))

        # --- Integrate ---
        vel += accel * dt
        depth += vel * dt
        if depth < -R:
            depth = -R
            if vel < 0:
                vel = 0.0
        if depth > max_depth:
            max_depth = depth

        # --- Print ---
        if i % print_every == 0:
            print(
                f"{i * dt:6.1f} {depth:9.4f} {measured_depth:9.4f} "
                f"{vel * 1000:9.2f} {est_velocity * 1000:11.2f} "
                f"{pbs_ml:8.2f} {pbs_ml:8.2f} {accel:9.5f}"
            )

    print("-" * 80)
    overshoot = (max_depth - 0.5) * 1000 if max_depth > 0.5 else 0.0
    print(
        f"Max depth: {max_depth:.4f}m | Final: {depth:.4f}m | "
        f"PBS: {pbs_ml:.2f}ml ({pbs_ml:.2f}g) | "
        f"Overshoot: {overshoot:.1f}mm"
    )


if __name__ == "__main__":
    main()
