# Autonomous Racing: Path Planning & Controls

My completed solutions to the **Path Planning & Controls (PPC) trainee module** for **IITB Racing Driverless**, a student-built autonomous electric race car that must complete a lap between blue and yellow track cones without collisions. The module is structured as four progressive checkpoints building toward a full closed-loop path-tracking pipeline, from raw waypoints to a car actually driving itself around a simulated track.

*(This repo is a fork of the team's [trainee module template](https://github.com/AyuwanC/ppc_trainee_module) — the checkpoints below are my implementations.)*

## Checkpoints

### 1. Interpolation (`checkpoint1_interpolation/`)
Given sparse cone/waypoint coordinates, build a smooth drivable path between them:
- Cubic spline interpolation implemented **from scratch** — hand-built the 4×(n-1) coefficient matrix and solved the boundary/continuity constraint system (value matching, first- and second-derivative continuity at knots, natural boundary conditions) directly, rather than calling a spline library
- Comparison against `scipy.interpolate.CubicSpline` to validate the manual implementation

### 2. Optimization & velocity profiling (`checkpoint2_optimization_velocity/`)
A smooth path isn't enough — the car needs to know how fast it can safely take each section:
- Arc-length parameterization of the path, then curvature computed from first/second derivatives of the parametric spline (`κ = |x'y'' − y'x''| / (x'² + y'²)^1.5`)
- Curvature-based velocity profile: lower speed limits on high-curvature (tight) sections, higher limits on straights

### 3. Controls (`checkpoint3_controls/`)
Getting the car to actually follow the planned path and velocity profile:
- **PID controller** for throttle, tracking the target velocity profile from checkpoint 2
- **Stanley controller** for steering, combining cross-track error and heading error to steer the car back onto the reference path

### 4. Full closed-loop implementation (`checkpoint4_the_final_implementation/`)
Everything integrated into one animated simulation:
- Kinematic bicycle model for vehicle dynamics
- PID throttle control + Stanley steering control running together in a closed loop against real cone-map waypoints (`waypoints.npy`, `blue_cones.npy`, `yellow_cones.npy`)
- Matplotlib animation of the car tracking the generated raceline in real time

## What this demonstrates
Interpolation and spline math implemented from first principles, curvature-based trajectory optimization, and two classical path-tracking controllers (PID, Stanley) integrated into a working closed-loop vehicle simulation — the core control stack behind an autonomous race car's ability to drive itself around a track.

## Stack

`NumPy` / `SciPy` · `Matplotlib` (animation)

## Running it

```bash
pip install -r requirements.txt
jupyter notebook checkpoint4_the_final_implementation/06_complete_implementation.ipynb
```

Each checkpoint's `.py`/`.ipynb` files are independently runnable and build toward the final integrated simulation.
