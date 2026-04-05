# Game Mode Design — Submarine Depth Control Simulator

## Engine Capabilities Summary

The submarine simulation engine provides:
- **1-DOF physics**: Vertical movement with realistic buoyancy, drag, and gravity
- **PID depth controller**: Automatic depth control with configurable gains (Kp, Ki, Kd)
- **Piston Ballast System (PBS)**: 0–30 ml capacity, 1.5 ml/s pump rate limit
- **Realistic sensors**: MS5837 depth sensor (50 Hz), MPU6050/ADXL355 accelerometers (200 Hz)
- **Sensor noise & bias**: Gaussian noise, quantization, startup bias calibration
- **Velocity fusion**: Complementary filter combining accelerometer + pressure data
- **Settle-time tracking**: Detects when submarine is within ±10 mm of target
- **Configurable hull**: Length (100–600 mm), diameter (20–100 mm), computed mass
- **Deterministic RNG**: Seeded pseudo-random number generator for reproducible runs

## Brainstormed Game Modes

### 1. ~~Depth Challenge~~ (removed)

Originally implemented, then removed in favor of the more challenging Fuel Economy mode.

---

### 2. Precision Dive

**Concept**: Reach an exact target depth with the smallest possible overshoot.

**Mechanics**:
- Single target depth given (e.g., 1.000 m)
- Score = inverse of overshoot (mm) × settle time multiplier
- Leaderboard tracks best overshoot per target depth
- Player tunes PID parameters (Kp, Ki, Kd sliders) to optimize approach

**Why it works**: The simulation already tracks overshoot and settle time. This mode turns PID tuning into a puzzle — aggressive gains reach target faster but overshoot more. Requires understanding the tradeoffs documented in the README (e.g., excessive Kd paradoxically increases overshoot due to integral windup).

---

### 3. Fuel Economy Challenge (⭐ Implemented)

**Concept**: Complete depth objectives with limited PBS pump budget using **manual buoyancy control** — no PID auto-pilot. The player directly controls the PBS pump level by clicking in the water.

**Mechanics**:
- **Manual PBS control**: PID auto-navigation is disabled. Mouse position in the water area continuously controls PBS pump level (top of water = 0 ml, bottom = 30 ml). The player must learn how PBS relates to buoyancy and manually guide the submarine.
- Player given a total "pump energy" budget (50 ml of cumulative PBS changes)
- All 5 target depths shown at once — player decides which order to complete them
- Each ml of PBS change costs 1 unit of budget
- Submarine must hold steady within ±30 mm with velocity < 10 mm/s for 3 sim seconds to capture a target
- Time limit: 600 sim seconds (~60 real seconds) — forces urgency
- Score = waypoint score (100 pts each, minus error penalty) + remaining fuel bonus (4 pts per ml saved)
- If budget runs out, PBS is frozen at current level — submarine drifts on momentum/buoyancy
- Canvas shows: neutral buoyancy line, PBS target cursor, waypoint markers with dwell progress, fuel gauge

**Why it works**: The player becomes the PID controller. To reach a target depth, they must: (1) increase PBS above neutral to sink, (2) reduce PBS toward neutral as they approach the target to decelerate, (3) fine-tune PBS near neutral to hover. Overshooting wastes fuel. Route planning, momentum management, and understanding neutral buoyancy are all critical. This is genuinely educational and challenging.

---

### 4. Sensor Degradation

**Concept**: Start with perfect sensors, then progressively degrade them during the mission.

**Mechanics**:
- Phase 1: Both sensors active (ADXL355 + MS5837) — reach targets easily
- Phase 2: Switch to MPU6050 (noisier accelerometer) — slight challenge
- Phase 3: Pressure sensor only (no accelerometer derivative) — PID struggles
- Phase 4: Accelerometer only (drift, no absolute reference) — maximum challenge
- Each phase has a target depth to reach before sensor degrades further

**Why it works**: Demonstrates the sensor fusion concepts from the README. Players learn why the complementary filter matters and what happens when sensor quality degrades. Educational and challenging.

---

### 5. Time Trial Speedrun

**Concept**: Reach target depth as fast as possible with automatic PID control.

**Mechanics**:
- Randomized target depth
- Timer starts on play, stops when settled within ±10 mm
- Player adjusts hull parameters (length, diameter) before starting
- Larger hulls have more drag (slower) but more stable (less overshoot)
- Smaller hulls are faster but harder to stabilize

**Why it works**: Turns hull configuration into a gameplay mechanic. Players explore the physics relationship between hull geometry, drag, mass, and settling behavior.

---

### 6. Obstacle Avoidance

**Concept**: Navigate through depth zones while avoiding "danger zones" at certain depths.

**Mechanics**:
- Horizontal bands of "danger" appear at various depths (visually marked as red zones)
- Submarine must pass through safe corridors between danger zones
- Target depths change periodically, requiring movement through the corridor
- Touching a danger zone = penalty (score deduction or game over)
- Safe corridor positions shift over time

**Why it works**: Adds spatial awareness and timing to the 1-DOF simulation. The submarine's momentum and overshoot become real hazards — you can't just set a target and wait, you need to manage approach velocity.

---

### 7. Multiplayer Leaderboard

**Concept**: Asynchronous competition — compare settle times and accuracy against other players.

**Mechanics**:
- Fixed scenario: same hull, same sensors, same target depth sequence
- Deterministic RNG ensures identical physics for all players
- Player's only input: when to change target depth (timing strategy)
- Score uploaded to leaderboard (total time to complete all targets)

**Why it works**: The seeded RNG (seed=42) guarantees identical sensor noise sequences, making direct comparison fair. Competition motivates understanding the physics.

---

## Implementation Priority

| Priority | Game Mode | Effort | Fun Factor | Educational Value |
|----------|-----------|--------|------------|-------------------|
| ~~1~~ | ~~Depth Challenge~~ | ~~Low~~ | ~~High~~ | ~~Medium~~ |
| 2 | Precision Dive | Low | Medium | High |
| ⭐ 3 | Fuel Economy | Medium | High | High |
| 4 | Sensor Degradation | Medium | Medium | Very High |
| 5 | Time Trial | Low | Medium | Medium |
| 6 | Obstacle Avoidance | High | High | Medium |
| 7 | Multiplayer Leaderboard | High | High | Low |

## Architecture Notes

The game mode system is designed to be non-invasive to the core simulation:

- `SubmarineSim` class remains unchanged — game logic is layered on top
- Game state (`fuelState` object) manages mode, waypoints, scoring, and UI
- Visual elements (waypoint markers, fuel gauge, dwell indicators) are drawn during the existing `renderFrame()` call
- Game controls are added to the existing controls panel
- The player controls the submarine using the same click-to-set-target and slider mechanisms

This means new game modes can be added by:
1. Defining waypoint/objective generation logic
2. Defining completion conditions
3. Defining scoring formulas
4. Adding any mode-specific visual elements
