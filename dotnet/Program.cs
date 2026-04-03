using System;

// =============================================================================
// Submarine PID Depth Control with Realistic Sensor Models
//
// Sensors modelled from datasheets:
//   - MS5837-30BA pressure sensor (depth): 0.016 mbar resolution, ~0.16mm depth
//   - MPU6050 accelerometer (±2g):  0.061 mg/LSB, 400 µg/√Hz noise density
//
// Velocity estimation: complementary filter fusing accelerometer integration
//                      (short-term) with depth-sensor differentiation (long-term)
// =============================================================================

// --- Gaussian RNG helper (Box–Muller) ---

public static class GaussianRng
{
    private static readonly Random _rng = new Random(42);
    private static bool _hasSpare;
    private static double _spare;

    public static double Next(double mean, double stdDev)
    {
        if (_hasSpare)
        {
            _hasSpare = false;
            return mean + stdDev * _spare;
        }
        double u, v, s;
        do
        {
            u = _rng.NextDouble() * 2.0 - 1.0;
            v = _rng.NextDouble() * 2.0 - 1.0;
            s = u * u + v * v;
        } while (s >= 1.0 || s == 0.0);

        s = Math.Sqrt(-2.0 * Math.Log(s) / s);
        _spare = v * s;
        _hasSpare = true;
        return mean + stdDev * u * s;
    }
}

// --- MS5837-30BA Pressure/Depth Sensor Model ---

public class MS5837Sensor
{
    // Datasheet specs (OSR 8192):
    //   Resolution: 0.016 mbar  →  depth step ≈ 0.016 / 98.1 m ≈ 0.163 mm
    //   RMS noise:  ~0.016 mbar →  σ ≈ 0.163 mm depth
    //   Conversion: ~18 ms per reading (P+T → ~36 ms → ~28 Hz; we use 50 Hz)
    //   Accuracy:   ±2 mbar    →  ±20.4 mm offset (factory bias, calibratable)

    const double PaPerMbarDiv = 98.1; // Pa per mbar for water depth: ρg/100

    const double ResolutionMbar = 0.016;
    const double NoiseSigmaMbar = 0.016;
    const double AccuracyMbar = 2.0;

    public double SamplePeriod { get; } = 0.020; // 50 Hz

    // Depth resolution in meters: 0.016 mbar / (9810 Pa/m * 0.01 mbar/Pa) = 0.016/98.1 m
    public double DepthResolutionM { get; } = ResolutionMbar / 98.1;

    double _noiseSigmaM;
    double _biasM;
    double _lastReading;
    double _timeSinceLastSample;
    bool _newSample;

    public MS5837Sensor()
    {
        _noiseSigmaM = NoiseSigmaMbar / 98.1;
        _biasM = (AccuracyMbar * (2.0 * new Random(123).NextDouble() - 1.0)) / 98.1;
        _lastReading = 0;
        _timeSinceLastSample = SamplePeriod; // force first sample immediately
        _newSample = false;
    }

    /// <summary>
    /// Returns (reading, isNewSample). Only updates at sensor sample rate.
    /// </summary>
    public (double depth, bool isNew) Read(double trueDepthM, double dt)
    {
        _timeSinceLastSample += dt;
        _newSample = false;
        if (_timeSinceLastSample >= SamplePeriod)
        {
            _timeSinceLastSample = 0;
            _newSample = true;

            double noisyDepth = trueDepthM + _biasM + GaussianRng.Next(0, _noiseSigmaM);
            _lastReading = Math.Round(noisyDepth / DepthResolutionM) * DepthResolutionM;
        }
        return (_lastReading, _newSample);
    }

    public double BiasM => _biasM;
}

// --- MPU6050 Accelerometer Model (±2g range, vertical axis only) ---

public class MPU6050Accel
{
    // Datasheet specs (±2g, DLPF_CFG=6 → BW=5Hz):
    //   Sensitivity: 16384 LSB/g
    //   Resolution:  0.061 mg/LSB → 0.000598 m/s²
    //   Noise density: 400 µg/√Hz, effective BW ~6.5 Hz → σ ≈ 1.02 mg ≈ 0.01 m/s²
    //   Zero-g bias: ±50 mg (X/Y), ±80 mg (Z)
    //   Sample rate: 200 Hz (DLPF enabled, SMPLRT_DIV=4)

    const double G_MPS2 = 9.80665;
    const double Sensitivity = 16384.0;
    const double LsbMps2 = G_MPS2 / Sensitivity; // ≈ 0.000598 m/s²

    const double NoiseDensity = 400e-6;  // g/√Hz
    const double EffectiveNoiseBw = 6.5; // Hz
    const double BiasMaxMg = 50.0;

    public double SamplePeriod { get; } = 0.005; // 200 Hz

    double _noiseSigmaMps2;
    double _biasMps2;
    double _lastReading;
    double _timeSinceLastSample;

    public MPU6050Accel()
    {
        _noiseSigmaMps2 = NoiseDensity * Math.Sqrt(EffectiveNoiseBw) * G_MPS2;
        _biasMps2 = (BiasMaxMg * (2.0 * new Random(456).NextDouble() - 1.0)) * 1e-3 * G_MPS2;
        _lastReading = 0;
        _timeSinceLastSample = SamplePeriod;
    }

    /// <summary>
    /// Returns measured net dynamic acceleration (m/s², positive = down).
    /// Gravity is NOT included — caller provides net dynamic accel only.
    /// </summary>
    public double Read(double trueAccelMps2, double dt)
    {
        _timeSinceLastSample += dt;
        if (_timeSinceLastSample >= SamplePeriod)
        {
            _timeSinceLastSample = 0;
            double noisy = trueAccelMps2 + _biasMps2 + GaussianRng.Next(0, _noiseSigmaMps2);
            _lastReading = Math.Round(noisy / LsbMps2) * LsbMps2;
        }
        return _lastReading;
    }

    public double BiasMps2 => _biasMps2;
    public double NoiseSigma => _noiseSigmaMps2;
}

// --- PID Controller (full P + I + D) ---

public class PidController
{
    public double Kp { get; set; }
    public double Ki { get; set; }
    public double Kd { get; set; }
    public double SetPoint { get; set; }

    public double OutputMin { get; set; } = double.MinValue;
    public double OutputMax { get; set; } = double.MaxValue;
    public double DeadBand { get; set; } = 0.0;

    private double _integral;

    public PidController(double kp, double ki, double kd, double setPoint)
    {
        Kp = kp; Ki = ki; Kd = kd; SetPoint = setPoint;
    }

    /// <summary>
    /// PID update. Velocity is provided externally (from sensor fusion)
    /// for the derivative term — derivative-on-measurement, not on error.
    /// </summary>
    public double Update(double processValue, double velocity, double dt)
    {
        double error = SetPoint - processValue;

        // Dead band on P and I only (not D — need full velocity signal)
        double errorForPI = (Math.Abs(error) <= DeadBand) ? 0 : error;

        double pTerm = Kp * errorForPI;

        // D term: negative velocity × Kd (derivative on measurement)
        double dTerm = -Kd * velocity;

        // Anti-windup: only integrate when output is not saturated,
        // or when error would reduce the integral (back-off)
        double candidateIntegral = _integral + errorForPI * dt;
        double candidateOutput = pTerm + Ki * candidateIntegral + dTerm;
        if (candidateOutput >= OutputMin && candidateOutput <= OutputMax)
        {
            _integral = candidateIntegral; // within bounds, integrate normally
        }
        else if (errorForPI * _integral < 0)
        {
            _integral = candidateIntegral; // error opposes integral → allow unwind
        }
        // else: saturated and error would make it worse → freeze integral

        double iTerm = Ki * _integral;

        return Math.Clamp(pTerm + iTerm + dTerm, OutputMin, OutputMax);
    }

    public void Reset() { _integral = 0; }
}

// --- Accelerometer Bias Calibrator ---
// At startup the submarine is nearly stationary (true accel ≈ 0).
// Average N samples to estimate the sensor bias, then subtract it.

public class AccelCalibrator
{
    double _sum;
    int _count;
    int _requiredSamples;
    double _estimatedBias;
    bool _done;

    public AccelCalibrator(double calibrationTimeSec, double samplePeriod)
    {
        _requiredSamples = Math.Max(1, (int)(calibrationTimeSec / samplePeriod));
        _sum = 0; _count = 0; _estimatedBias = 0; _done = false;
    }

    /// <summary>Feed raw accel sample. Returns true when calibration is complete.</summary>
    public bool AddSample(double rawAccel)
    {
        if (_done) return true;
        _sum += rawAccel;
        _count++;
        if (_count >= _requiredSamples)
        {
            _estimatedBias = _sum / _count;
            _done = true;
        }
        return _done;
    }

    public double EstimatedBias => _estimatedBias;
    public bool IsDone => _done;
}

// --- Complementary Filter Velocity Estimator ---

public class VelocityFusion
{
    // Fuses accelerometer integration (short-term, bias-calibrated)
    // with depth-sensor differentiation (long-term, drift-free).
    //
    // Predict:  v_fused += calibratedAccel * dt
    // Correct:  on each depth sample, blend toward depth-derived velocity
    //           β = Δt / (τ + Δt) where Δt = depth sample interval

    double _tau;
    double _fusedVelocity;
    double _prevDepth;
    double _depthAge;
    bool _firstDepth;

    public VelocityFusion(double tau = 0.5)
    {
        _tau = tau;
        _fusedVelocity = 0;
        _prevDepth = 0;
        _depthAge = 0;
        _firstDepth = true;
    }

    public double Update(double calibratedAccel, double depth,
                         bool newDepthSample, double dt)
    {
        // Predict: integrate calibrated accelerometer
        _fusedVelocity += calibratedAccel * dt;
        _depthAge += dt;

        // Correct: when depth sensor updates, blend toward depth-derived velocity
        if (newDepthSample)
        {
            if (_firstDepth)
            {
                _prevDepth = depth;
                _firstDepth = false;
            }
            else if (_depthAge > 1e-9)
            {
                double vDepth = (depth - _prevDepth) / _depthAge;
                _prevDepth = depth;

                double beta = _depthAge / (_tau + _depthAge);
                _fusedVelocity = (1.0 - beta) * _fusedVelocity + beta * vDepth;
            }
            _depthAge = 0;
        }

        return _fusedVelocity;
    }

    public double Velocity => _fusedVelocity;
}

// --- Submarine Simulation ---

class Program
{
    const double G = 9.81;
    const double Rho = 1000.0;

    const double L = 0.30;
    const double R = 0.025;
    const double HullMass = 0.570;

    const double PbsMaxMl = 30.0;

    const double Cd = 1.0;
    const double DragArea = L * 2 * R;

    static double SubmergedVolume(double d)
    {
        if (d >= R) return Math.PI * R * R * L;
        if (d <= -R) return 0.0;
        double area = R * R * Math.Acos(-d / R) + d * Math.Sqrt(R * R - d * d);
        return area * L;
    }

    static double FindEquilibrium(double totalMassKg)
    {
        double lo = -R, hi = R;
        for (int i = 0; i < 200; i++)
        {
            double mid = (lo + hi) / 2.0;
            double buoyancy = Rho * SubmergedVolume(mid);
            if (buoyancy < totalMassKg) lo = mid; else hi = mid;
        }
        return (lo + hi) / 2.0;
    }

    static void Main()
    {
        double neutralPbsMl = (Rho * Math.PI * R * R * L - HullMass) * 1e3;

        double initPbsMl = neutralPbsMl - 1.0;
        double initMass = HullMass + initPbsMl * 1e-3;
        double d0 = FindEquilibrium(initMass);

        // --- Sensors ---
        var depthSensor = new MS5837Sensor();
        var accelSensor = new MPU6050Accel();

        // Accelerometer bias calibration: 0.5s at startup while stationary
        const double CalibTimeSec = 0.5;
        var accelCalib = new AccelCalibrator(CalibTimeSec, accelSensor.SamplePeriod);

        // Velocity fusion: τ = 0.5s complementary filter
        const double FusionTau = 0.5;
        var velFusion = new VelocityFusion(tau: FusionTau);

        const double MaxPumpRate = 1.5;

        // --- PID ---
        var pid = new PidController(
            kp: 10.0,
            ki: 0.1,
            kd: 80.0,
            setPoint: 0.5
        )
        {
            OutputMin = -neutralPbsMl,
            OutputMax = PbsMaxMl - neutralPbsMl,
            DeadBand = 0.001
        };

        double dt = 0.001;
        double simTime = 60.0;
        int steps = (int)(simTime / dt);
        int printEvery = (int)(1.0 / dt);

        double depth = d0, vel = 0, pbsMl = initPbsMl;
        double maxDepth = 0;

        // Track previous accel reading to detect new samples for calibration
        double prevRawAccel = double.NaN;

        // --- Header ---
        Console.WriteLine("=== Submarine PID Depth Control ===");
        Console.WriteLine($"Cylinder: {L * 100:F0}cm x dia {R * 200:F0}cm, Hull: {HullMass * 1000:F0}g");
        Console.WriteLine($"Volume: {Math.PI * R * R * L * 1e6:F1}ml, Neutral PBS: {neutralPbsMl:F1}ml");
        Console.WriteLine($"Init: PBS={initPbsMl:F1}ml, depth={d0 * 1000:F1}mm");
        Console.WriteLine();
        Console.WriteLine($"Depth sensor: MS5837-30BA (OSR 8192)");
        Console.WriteLine($"  Resolution: {depthSensor.DepthResolutionM * 1000:F3}mm, " +
                          $"Rate: {1.0 / depthSensor.SamplePeriod:F0}Hz, " +
                          $"Bias: {depthSensor.BiasM * 1000:F2}mm");
        Console.WriteLine($"Accel: MPU6050 (±2g, DLPF 5Hz)");
        Console.WriteLine($"  True bias: {accelSensor.BiasMps2 * 1000:F2}mm/s², " +
                          $"Noise σ: {accelSensor.NoiseSigma * 1000:F2}mm/s²");
        Console.WriteLine($"  Calibration: {CalibTimeSec:F1}s at startup");
        Console.WriteLine($"Pump: max {MaxPumpRate:F1} ml/s");
        Console.WriteLine($"Target: 0.500m");
        Console.WriteLine();
        Console.WriteLine($"PID: Kp={pid.Kp}, Ki={pid.Ki}, Kd={pid.Kd}, DeadBand={pid.DeadBand * 1000:F1}mm");
        Console.WriteLine($"Velocity fusion τ={FusionTau:F1}s");
        Console.WriteLine();
        Console.WriteLine($"{"t(s)",6} {"True(m)",9} {"Sens(m)",9} {"v(mm/s)",9} {"vEst(mm/s)",11} {"PBS(ml)",8} {"H2O(g)",8} {"a(m/s²)",9}");
        Console.WriteLine(new string('-', 80));

        for (int i = 0; i <= steps; i++)
        {
            // --- Physics ---
            double totalMass = HullMass + pbsMl * 1e-3;
            double weight = totalMass * G;
            double buoyancy = Rho * G * SubmergedVolume(depth);
            double drag = 0.5 * Rho * Cd * DragArea * vel * Math.Abs(vel);
            double accel = (weight - buoyancy - drag) / totalMass;

            // --- Sensor readings ---
            var (measuredDepth, newDepthSample) = depthSensor.Read(depth, dt);
            double rawAccel = accelSensor.Read(accel, dt);

            // Detect new accelerometer sample (value changed = new conversion)
            bool newAccelSample = (rawAccel != prevRawAccel);
            prevRawAccel = rawAccel;

            // --- Accelerometer bias calibration ---
            if (!accelCalib.IsDone)
            {
                if (newAccelSample)
                    accelCalib.AddSample(rawAccel);

                // During calibration: no PID, hold PBS constant, no velocity estimation
                // Just integrate physics
                vel += accel * dt;
                depth += vel * dt;
                if (depth < -R) { depth = -R; if (vel < 0) vel = 0; }
                if (depth > maxDepth) maxDepth = depth;

                if (i % printEvery == 0)
                {
                    Console.WriteLine(
                        $"{i * dt,6:F1} {depth,9:F4} {measuredDepth,9:F4} " +
                        $"{vel * 1000,9:F2} {"[CALIB]",11} " +
                        $"{pbsMl,8:F2} {pbsMl,8:F2} {accel,9:F5}");
                }

                if (accelCalib.IsDone)
                {
                    Console.WriteLine($"  >>> Accel calibration done: estimated bias = " +
                                      $"{accelCalib.EstimatedBias * 1000:F2} mm/s² " +
                                      $"(true: {accelSensor.BiasMps2 * 1000:F2} mm/s²)");
                }
                continue;
            }

            // --- Calibrated accelerometer ---
            double calibratedAccel = rawAccel - accelCalib.EstimatedBias;

            // --- Velocity fusion ---
            double estVelocity = velFusion.Update(calibratedAccel, measuredDepth,
                                                   newDepthSample, dt);

            // --- PID ---
            double correction = pid.Update(measuredDepth, estVelocity, dt);
            double targetPbs = Math.Clamp(neutralPbsMl + correction, 0.0, PbsMaxMl);

            double maxChange = MaxPumpRate * dt;
            pbsMl += Math.Clamp(targetPbs - pbsMl, -maxChange, maxChange);
            pbsMl = Math.Clamp(pbsMl, 0.0, PbsMaxMl);

            // --- Integrate ---
            vel += accel * dt;
            depth += vel * dt;
            if (depth < -R) { depth = -R; if (vel < 0) vel = 0; }
            if (depth > maxDepth) maxDepth = depth;

            // --- Print ---
            if (i % printEvery == 0)
            {
                Console.WriteLine(
                    $"{i * dt,6:F1} {depth,9:F4} {measuredDepth,9:F4} " +
                    $"{vel * 1000,9:F2} {estVelocity * 1000,11:F2} " +
                    $"{pbsMl,8:F2} {pbsMl,8:F2} {accel,9:F5}");
            }
        }

        Console.WriteLine(new string('-', 80));
        Console.WriteLine($"Max depth: {maxDepth:F4}m | Final: {depth:F4}m | " +
                          $"PBS: {pbsMl:F2}ml ({pbsMl:F2}g) | " +
                          $"Overshoot: {(maxDepth > 0.5 ? (maxDepth - 0.5) * 1000 : 0):F1}mm");
    }
}
