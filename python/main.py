"""
Submarine PID Depth Control with Realistic Sensor Models

Sensors modelled from datasheets:
  - MS5837-30BA pressure sensor (depth): 0.016 mbar resolution, ~0.16mm depth
  - MPU6050 accelerometer (+-2g):  0.061 mg/LSB, 400 ug/sqrt(Hz) noise density

Velocity estimation: complementary filter fusing accelerometer integration
                     (short-term) with depth-sensor differentiation (long-term)
"""

import argparse
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


# --- ADXL355 Accelerometer Model (+-2g range, vertical axis only) ---

class ADXL355Accel:
    """
    Datasheet specs (+-2g, digital LPF comparable to MPU6050 BW~5Hz config):
      Sensitivity:   256000 LSB/g  (20-bit output)
      Resolution:    3.9 ug/LSB -> 0.0000383 m/s2
      Noise density: 25 ug/sqrt(Hz), effective BW ~6.5 Hz -> sigma ~ 0.064 mg ~ 0.000625 m/s2
      Zero-g bias:   +-10 mg (typ)
      Sample rate:   200 Hz (ODR configurable, matched to simulation)
      Offset tempco: 0.15 mg/C (max)
    """

    _G_MPS2 = 9.80665
    _SENSITIVITY = 256000.0          # 20-bit at +-2g
    _LSB_MPS2 = _G_MPS2 / _SENSITIVITY   # ~ 0.0000383 m/s2

    _NOISE_DENSITY = 25e-6               # g/sqrt(Hz)  -- 16x lower than MPU6050
    _EFFECTIVE_NOISE_BW = 6.5            # Hz (comparable filter config)
    _BIAS_MAX_MG = 10.0                  # +-10 mg (5x lower than MPU6050)

    def __init__(self) -> None:
        self.sample_period: float = 0.005  # 200 Hz

        self._noise_sigma_mps2: float = (
            self._NOISE_DENSITY * math.sqrt(self._EFFECTIVE_NOISE_BW) * self._G_MPS2
        )
        self._bias_mps2: float = (
            self._BIAS_MAX_MG * (2.0 * random.Random(789).random() - 1.0)
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


# --- Accelerometer model registry ---

ACCEL_MODELS: dict[str, type] = {
    "mpu6050": MPU6050Accel,
    "adxl355": ADXL355Accel,
}


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

        # Dead band: suppress ALL terms and drain integral when within tolerance
        if abs(error) <= self.dead_band:
            # Exponentially decay integral toward zero to prevent accumulated
            # bias from pushing PBS once the submarine is within tolerance.
            self._integral *= 0.99
            return 0.0

        p_term = self.kp * error

        # D term: negative velocity * Kd (derivative on measurement)
        d_term = -self.kd * velocity

        # Anti-windup: only integrate when output is not saturated,
        # or when error would reduce the integral (back-off)
        candidate_integral = self._integral + error * dt
        candidate_output = p_term + self.ki * candidate_integral + d_term
        if self.output_min <= candidate_output <= self.output_max:
            self._integral = candidate_integral  # within bounds
        elif error * self._integral < 0:
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

    Supports three modes:
      "both"     – complementary filter (default, original behaviour)
      "pressure" – velocity derived only from depth differentiation
      "accel"    – velocity derived only from accelerometer integration
    """

    VALID_MODES = ("both", "pressure", "accel")

    def __init__(self, tau: float = 0.5, mode: str = "both") -> None:
        if mode not in self.VALID_MODES:
            raise ValueError(f"mode must be one of {self.VALID_MODES}, got {mode!r}")
        self._tau = tau
        self._mode = mode
        self._fused_velocity: float = 0.0
        self._prev_depth: float = 0.0
        self._depth_age: float = 0.0
        self._first_depth: bool = True

    # --- mode property ---

    @property
    def mode(self) -> str:
        return self._mode

    @mode.setter
    def mode(self, value: str) -> None:
        if value not in self.VALID_MODES:
            raise ValueError(f"mode must be one of {self.VALID_MODES}, got {value!r}")
        self._mode = value

    # --- update ---

    def update(
        self,
        calibrated_accel: float,
        depth: float,
        new_depth_sample: bool,
        dt: float,
    ) -> float:
        mode = self._mode

        if mode == "accel":
            # Accelerometer-only: integrate accel, no depth correction
            self._fused_velocity += calibrated_accel * dt
            return self._fused_velocity

        if mode == "pressure":
            # Pressure-only: velocity from depth differentiation
            self._depth_age += dt
            if new_depth_sample:
                if self._first_depth:
                    self._prev_depth = depth
                    self._first_depth = False
                elif self._depth_age > 1e-9:
                    self._fused_velocity = (
                        (depth - self._prev_depth) / self._depth_age
                    )
                    self._prev_depth = depth
                self._depth_age = 0.0
            return self._fused_velocity

        # mode == "both": complementary filter (original behaviour)
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

# Hull defaults (user-facing units: mm)
DEFAULT_HULL_LENGTH_MM = 300.0      # mm
DEFAULT_HULL_DIAMETER_MM = 50.0     # mm

PBS_MAX_ML = 30.0

CD = 1.0


def submerged_volume(d: float, R: float, L: float) -> float:
    """Volume of a horizontal cylinder submerged to centre-depth *d*."""
    if d >= R:
        return math.pi * R * R * L
    if d <= -R:
        return 0.0
    area = R * R * math.acos(-d / R) + d * math.sqrt(R * R - d * d)
    return area * L


def find_equilibrium(total_mass_kg: float, R: float, L: float) -> float:
    """Binary-search for centre-depth where buoyancy equals weight."""
    lo, hi = -R, R
    for _ in range(200):
        mid = (lo + hi) / 2.0
        buoyancy = RHO * submerged_volume(mid, R, L)
        if buoyancy < total_mass_kg:
            lo = mid
        else:
            hi = mid
    return (lo + hi) / 2.0


class SubmarineSim:
    """Encapsulates the full submarine depth-control simulation state.

    Usage:
        sim = SubmarineSim(target_depth=0.5)
        for _ in range(N):
            sim.step()    # advances by sim.dt seconds
    """

    def __init__(
        self,
        target_depth: float = 0.5,
        dt: float = 0.001,
        calib_time_sec: float = 0.5,
        fusion_tau: float = 0.5,
        max_pump_rate: float = 1.5,
        kp: float = 10.0,
        ki: float = 0.1,
        kd: float = 80.0,
        dead_band: float = 0.010,
        sensor_mode: str = "both",
        accel_model: str = "mpu6050",
        hull_length_mm: float = DEFAULT_HULL_LENGTH_MM,
        hull_diameter_mm: float = DEFAULT_HULL_DIAMETER_MM,
    ) -> None:
        if accel_model not in ACCEL_MODELS:
            raise ValueError(
                f"accel_model must be one of {list(ACCEL_MODELS)}, got {accel_model!r}"
            )
        self.dt = dt
        self.max_pump_rate = max_pump_rate
        self._accel_model_name: str = accel_model

        # Hull geometry (convert mm → SI)
        self._L: float = hull_length_mm * 1e-3           # m
        self._R: float = hull_diameter_mm * 0.5e-3        # m

        # Drag cross-section (side projection of horizontal cylinder)
        self._drag_area: float = self._L * 2 * self._R    # m²

        # Neutral buoyancy at mid-range of ballast tank
        self.neutral_pbs_ml: float = PBS_MAX_ML / 2.0

        # Hull mass: lighter than water so that neutral_pbs_ml of ballast
        # brings it to exact neutral buoyancy (density ratio < 1)
        self._M: float = (
            RHO * math.pi * self._R ** 2 * self._L
            - self.neutral_pbs_ml * 1e-3
        )  # kg

        # Initial conditions — slightly positively buoyant, at surface
        init_pbs_ml = self.neutral_pbs_ml - 1.0
        init_mass = self._M + init_pbs_ml * 1e-3
        d0 = find_equilibrium(init_mass, self._R, self._L)

        # State
        self.time: float = 0.0
        self.depth: float = d0
        self.velocity: float = 0.0
        self.pbs_ml: float = init_pbs_ml
        self.max_depth: float = 0.0
        self.measured_depth: float = 0.0
        self.est_velocity: float = 0.0
        self.accel: float = 0.0
        self.calibrating: bool = True

        # Sensors
        self._depth_sensor = MS5837Sensor()
        self._accel_sensor = ACCEL_MODELS[accel_model]()

        # Calibrator
        self._accel_calib = AccelCalibrator(calib_time_sec, self._accel_sensor.sample_period)

        # Velocity fusion
        self._vel_fusion = VelocityFusion(tau=fusion_tau, mode=sensor_mode)

        # PID
        self._pid = PidController(
            kp=kp,
            ki=ki,
            kd=kd,
            set_point=target_depth,
            output_min=-self.neutral_pbs_ml,
            output_max=PBS_MAX_ML - self.neutral_pbs_ml,
            dead_band=dead_band,
        )

        # Internal bookkeeping
        self._prev_raw_accel: float = float("nan")

    # --- Public API ---

    @property
    def target_depth(self) -> float:
        return self._pid.set_point

    @target_depth.setter
    def target_depth(self, value: float) -> None:
        self._pid.set_point = value

    @property
    def sensor_mode(self) -> str:
        return self._vel_fusion.mode

    @sensor_mode.setter
    def sensor_mode(self, value: str) -> None:
        self._vel_fusion.mode = value

    @property
    def accel_model(self) -> str:
        return self._accel_model_name

    def step(self) -> None:
        """Advance simulation by one dt step."""
        dt = self.dt

        # --- Physics ---
        weight = (self._M + self.pbs_ml * 1e-3) * G
        buoyancy = RHO * G * submerged_volume(self.depth, self._R, self._L)
        drag = (
            0.5 * RHO * CD * self._drag_area
            * self.velocity * abs(self.velocity)
        )
        self.accel = (weight - buoyancy - drag) / self._M

        # --- Sensor readings ---
        self.measured_depth, new_depth_sample = self._depth_sensor.read(self.depth, dt)
        raw_accel = self._accel_sensor.read(self.accel, dt)

        new_accel_sample = raw_accel != self._prev_raw_accel
        self._prev_raw_accel = raw_accel

        # --- Calibration phase ---
        if not self._accel_calib.is_done:
            if new_accel_sample:
                self._accel_calib.add_sample(raw_accel)

            self.velocity += self.accel * dt
            self.depth += self.velocity * dt
            self._clamp()
            self.calibrating = not self._accel_calib.is_done
            self.time += dt
            return

        self.calibrating = False

        # --- Calibrated accel + fusion ---
        calibrated_accel = raw_accel - self._accel_calib.estimated_bias
        self.est_velocity = self._vel_fusion.update(
            calibrated_accel, self.measured_depth, new_depth_sample, dt,
        )

        # --- PID ---
        correction = self._pid.update(self.measured_depth, self.est_velocity, dt)
        target_pbs = max(0.0, min(PBS_MAX_ML, self.neutral_pbs_ml + correction))

        max_change = self.max_pump_rate * dt
        self.pbs_ml += max(-max_change, min(max_change, target_pbs - self.pbs_ml))
        self.pbs_ml = max(0.0, min(PBS_MAX_ML, self.pbs_ml))

        # --- Integrate ---
        self.velocity += self.accel * dt
        self.depth += self.velocity * dt
        self._clamp()
        self.time += dt

    # --- Helpers ---

    def _clamp(self) -> None:
        if self.depth < -self._R:
            self.depth = -self._R
            if self.velocity < 0:
                self.velocity = 0.0
        if self.depth > self.max_depth:
            self.max_depth = self.depth


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Submarine PID depth control simulation",
    )
    parser.add_argument(
        "--accel",
        choices=list(ACCEL_MODELS),
        default="mpu6050",
        help="accelerometer model (default: mpu6050)",
    )
    parser.add_argument(
        "--mode",
        choices=list(VelocityFusion.VALID_MODES),
        default="both",
        help="sensor fusion mode (default: both)",
    )
    parser.add_argument(
        "--target",
        type=float,
        default=0.5,
        help="target depth in meters (default: 0.5)",
    )
    parser.add_argument(
        "--time",
        type=float,
        default=60.0,
        dest="sim_time",
        help="simulation duration in seconds (default: 60)",
    )
    parser.add_argument(
        "--length",
        type=float,
        default=DEFAULT_HULL_LENGTH_MM,
        help=f"hull length in mm (default: {DEFAULT_HULL_LENGTH_MM:.0f})",
    )
    parser.add_argument(
        "--diameter",
        type=float,
        default=DEFAULT_HULL_DIAMETER_MM,
        help=f"hull diameter in mm (default: {DEFAULT_HULL_DIAMETER_MM:.0f})",
    )
    args = parser.parse_args()

    sim = SubmarineSim(
        target_depth=args.target,
        accel_model=args.accel,
        sensor_mode=args.mode,
        hull_length_mm=args.length,
        hull_diameter_mm=args.diameter,
    )

    sim_time = args.sim_time
    steps = int(sim_time / sim.dt)
    print_every = int(1.0 / sim.dt)

    # --- Header ---
    volume_ml = math.pi * sim._R ** 2 * sim._L * 1e6
    print("=== Submarine PID Depth Control ===")
    print(
        f"Hull: {args.length:.0f}mm x dia {args.diameter:.0f}mm, "
        f"mass {sim._M * 1000:.0f}g (computed)"
    )
    print(
        f"Volume: {volume_ml:.1f}ml, "
        f"Neutral PBS: {sim.neutral_pbs_ml:.1f}ml, "
        f"Drag area: {sim._drag_area:.5f}m2"
    )
    print(f"Init: PBS={sim.pbs_ml:.1f}ml, depth={sim.depth * 1000:.1f}mm")
    print()
    print("Depth sensor: MS5837-30BA (OSR 8192)")
    print(
        f"  Resolution: {sim._depth_sensor.depth_resolution_m * 1000:.3f}mm, "
        f"Rate: {1.0 / sim._depth_sensor.sample_period:.0f}Hz, "
        f"Bias: {sim._depth_sensor.bias_m * 1000:.2f}mm"
    )
    print(f"Accel: {args.accel.upper()}")
    print(
        f"  True bias: {sim._accel_sensor.bias_mps2 * 1000:.2f}mm/s2, "
        f"Noise sigma: {sim._accel_sensor.noise_sigma * 1000:.2f}mm/s2"
    )
    print(f"Sensor mode: {args.mode}")
    print(f"Pump: max {sim.max_pump_rate:.1f} ml/s")
    print(f"Target: {sim.target_depth:.3f}m")
    print()
    print(
        f"PID: Kp={sim._pid.kp}, Ki={sim._pid.ki}, Kd={sim._pid.kd}, "
        f"DeadBand={sim._pid.dead_band * 1000:.1f}mm"
    )
    print()
    print(
        f"{'t(s)':>6} {'True(m)':>9} {'Sens(m)':>9} {'v(mm/s)':>9} "
        f"{'vEst(mm/s)':>11} {'PBS(ml)':>8} {'H2O(g)':>8} {'a(m/s2)':>9}"
    )
    print("-" * 80)

    calib_msg_printed = False

    for i in range(steps + 1):
        if i % print_every == 0:
            if sim.calibrating:
                print(
                    f"{sim.time:6.1f} {sim.depth:9.4f} {sim.measured_depth:9.4f} "
                    f"{sim.velocity * 1000:9.2f} {'[CALIB]':>11} "
                    f"{sim.pbs_ml:8.2f} {sim.pbs_ml:8.2f} {sim.accel:9.5f}"
                )
            else:
                print(
                    f"{sim.time:6.1f} {sim.depth:9.4f} {sim.measured_depth:9.4f} "
                    f"{sim.velocity * 1000:9.2f} {sim.est_velocity * 1000:11.2f} "
                    f"{sim.pbs_ml:8.2f} {sim.pbs_ml:8.2f} {sim.accel:9.5f}"
                )

        was_calibrating = sim.calibrating
        sim.step()

        if was_calibrating and not sim.calibrating and not calib_msg_printed:
            print(
                f"  >>> Accel calibration done: estimated bias = "
                f"{sim._accel_calib.estimated_bias * 1000:.2f} mm/s2 "
                f"(true: {sim._accel_sensor.bias_mps2 * 1000:.2f} mm/s2)"
            )
            calib_msg_printed = True

    print("-" * 80)
    overshoot = (sim.max_depth - sim.target_depth) * 1000 if sim.max_depth > sim.target_depth else 0.0
    print(
        f"Max depth: {sim.max_depth:.4f}m | Final: {sim.depth:.4f}m | "
        f"PBS: {sim.pbs_ml:.2f}ml ({sim.pbs_ml:.2f}g) | "
        f"Overshoot: {overshoot:.1f}mm"
    )


if __name__ == "__main__":
    main()
