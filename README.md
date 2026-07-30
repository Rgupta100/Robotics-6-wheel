# my_swerve_control — 6-wheel swerve drive kinematics (ROS 2 Humble)

Inverse kinematics for a six-module swerve drive: given a chassis velocity
command, solve every wheel's steering angle and drive speed so all six roll
about a single instantaneous centre of rotation (ICR) instead of scrubbing
sideways.

Written for the University of Waterloo Robotics Team (Mars rover), Feb–Mar 2026.

## The solver

For a chassis twist `(Vx, Vy, ω)` and an arbitrary pivot point `(px, py)`,
each module `i` at body-frame position `(xᵢ, yᵢ)` gets:

```
dx = xᵢ − px
dy = yᵢ − py

vx = Vx − ω·dy          # velocity of this module in the body frame
vy = Vy + ω·dx

θᵢ = atan2(vy, vx)      # steering angle
|v|ᵢ = √(vx² + vy²)     # module ground speed
ωₘᵢ = |v|ᵢ / r          # drive motor angular speed
```

Because each module is steered onto its own velocity vector, every wheel's
rolling direction is tangent to a circle about the same ICR — which is what
makes the motion zero-scrubbing. The `px, py` term generalises this beyond
centre-of-mass rotation: the robot can pivot about any point, including a point
outside its own footprint.

Module layout (`swerve_optimizer.cpp`), 0.6 m × 0.4 m:

```
x = { 0.3,  0.0, -0.3,   0.3,  0.0, -0.3 }   # front, mid, back
y = { 0.2,  0.2,  0.2,  -0.2, -0.2, -0.2 }   # left ×3, right ×3
```

## Module optimisation

Solving the ICR gives a *raw* answer that is often the hard way round. Three
corrections sit on top of it:

**Shortest-path flip.** If a module would have to swing more than 90° to reach
its target heading, point it the opposite way and drive the wheel backwards
instead. The ground velocity vector is identical and no module ever rotates more
than a quarter turn. A module asked for 170° goes to −10° at reversed speed.

**Singularity hold.** `atan2(0, 0)` is undefined and returns 0, which would snap
every module to straight-ahead the instant the robot stops. Below 1e-3 the
previous angle is held and speed is zeroed, so the modules stay where they are.

**Speed desaturation.** If any module is asked for more than `max_motor_speed`,
*all six* are scaled by the same factor. Clipping one module alone would change
the direction the robot actually travels.

## Layout

```
src/my_swerve_control/
  src/swerve_optimizer.cpp    ROS 2 node — /cmd_vel in, steer + drive commands out
  src/Testing.cpp             standalone algorithm sandbox, no ROS dependency
  test/test_kinematics.cpp    19 assertions over the maths, no ROS dependency
  CMakeLists.txt
  package.xml
```

`swerve_optimizer.cpp` subscribes to `/cmd_vel` and runs a 50 Hz timer,
publishing `Float64MultiArray` to `/swerve_steer_controller/commands` and
`/swerve_drive_controller/commands`. It fails safe to a stop if no command
arrives within `cmd_timeout`.

`Testing.cpp` is where the maths was worked out — it compiles with plain `g++`
and prints the six (angle, velocity) pairs for a given twist, which made it
possible to check the solver by hand before involving Gazebo.

### Parameters

| Name | Default | Meaning |
|---|---|---|
| `wheel_radius` | 0.05 | metres, converts ground speed to motor rad/s |
| `pivot_x`, `pivot_y` | 0.0, 0.0 | pivot point in the body frame |
| `max_motor_speed` | 20.0 | rad/s, desaturation ceiling |
| `cmd_timeout` | 0.5 | seconds before `/cmd_vel` silence stops the robot |

## Tests

`test/test_kinematics.cpp` covers angle wrapping, the flip, the singularity
hold, desaturation, and the zero-scrubbing property itself — including a
73 × 73 sweep of every start/target angle pair checking that the flip never
exceeds a quarter turn *and* leaves the ground velocity vector unchanged.

Zero-scrubbing is asserted directly: for pure rotation, and for rotation about
an arbitrary off-centre pivot, every module's heading must be perpendicular to
its own radius from the ICR. That is what "no wheel drags sideways" means
geometrically.

Standalone, no ROS needed:

```bash
g++ -std=c++17 -Wall -Wextra -O2 src/my_swerve_control/test/test_kinematics.cpp -o test_kinematics
./test_kinematics
```

Or through colcon: `colcon test --packages-select my_swerve_control`.

All 19 assertions pass as of 2026-07-29.

## Simulation

`swerve_sim` provides the Gazebo side: a six-module URDF, `ros2_control`
configuration, a flat world, and a launch file that chains everything off the
spawn event.

```bash
ros2 launch swerve_sim sim.launch.py
ros2 launch swerve_sim sim.launch.py gui:=false     # headless
ros2 launch swerve_sim sim.launch.py solver:=false  # drive it by hand
```

Then drive it:

```bash
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
  '{linear: {x: 0.4, y: 0.2}, angular: {z: 0.3}}' -r 20
```

> [!warning] Joint order is load-bearing
> `ForwardCommandController` maps `Float64MultiArray[i]` onto `joints[i]`. The
> `joints:` lists in `swerve_sim/config/controllers.yaml` are in the same order
> the node fills its arrays — front_left, mid_left, back_left, front_right,
> mid_right, back_right. Reorder one without the other and the robot still
> moves, just driving the wrong wheels. It does not look like a failure.

## Scrub benchmark

Scrub is the component of a wheel's ground velocity perpendicular to the way it
is pointing — a wheel not pointing where it is going has to drag sideways, which
is wasted energy and tyre wear. It is the thing a swerve drive exists to
eliminate.

An *ideal* swerve has exactly zero scrub by construction, so "100% reduction"
would be true and meaningless. Real modules cannot snap instantly, so
`benchmark/scrub_benchmark.cpp` rate-limits the steering and measures the scrub
that survives during transients. The baseline is a fixed-heading (skid-steer)
rover — what the platform does with no steering solver.

```bash
g++ -std=c++17 -O2 benchmark/scrub_benchmark.cpp -o scrub_benchmark
./scrub_benchmark
./scrub_benchmark --rate 2 --csv trace.csv
```

Over a 24 s path (straight → figure-eight → strafe → spin in place), at a
steering rate limit of 6 rad/s and dt = 1 ms:

| | scrub (m) | roll (m) |
|---|---|---|
| fixed heading (baseline) | 25.09 | 63.36 |
| swerve (rate-limited) | 0.63 | 79.28 |

**97.5% scrub reduction.** Sensitivity, since the steering rate is the
assumption carrying the result:

| steering rate (rad/s) | 1 | 2 | 4 | 6 | 10 | 20 |
|---|---|---|---|---|---|---|
| scrub reduction | 85.0% | 92.5% | 96.3% | 97.5% | 98.5% | 99.3% |

Stable to 0.1% across timesteps from 10 ms down to 0.5 ms. Even a slow 1 rad/s
module removes 85% of the scrub.

The residual is transient only — it occurs while modules are still slewing
toward their commanded heading, which is exactly what you would expect and is
why the number is not 100%.

## Build

```bash
cd swerve_ws
colcon build
source install/setup.bash
ros2 run my_swerve_control swerve_optimizer
```

`Testing.cpp` standalone:

```bash
g++ src/my_swerve_control/src/Testing.cpp -o testing && ./testing
```

## Status — what is and isn't finished

Read this before assuming the node is production-ready.

- **The simulation has been rebuilt but never run.** The original launch files,
  URDF, `ros2_control` config and Gazebo world were destroyed in the 2026-07-18
  reset (see below). `swerve_sim` is a fresh reconstruction written 2026-07-29
  on a machine with **no ROS 2 and no WSL**, so it has never been through
  `colcon build` or Gazebo. Treat it as unverified until it comes up: expect to
  fix package paths, plugin names and controller spawn timing on first run.
- **Zero-scrubbing is proved geometrically, not empirically.** The test asserts
  every module is perpendicular to its ICR radius, which is the property itself.
  It has not been re-observed in Gazebo since the reset.
- **The benchmark is analytic, not simulated.** It integrates the kinematics
  with a steering rate limit; it does not model tyre slip, actuator lag beyond
  the rate limit, mass, or ground compliance. It is an upper bound on how much
  scrub the solver removes, and the number should be re-measured in Gazebo
  before being quoted as a physical result.
- **Units are mixed and it's a trap.** `Testing.cpp` works in degrees
  (`* 180 / M_PI`); `swerve_optimizer.cpp` and the tests work in radians. Don't
  copy a constant from one to the other.
- `Testing.cpp` is kept as the original working sandbox. Its versions of the
  three corrections are the degree-based originals; the radian ports in the node
  are what actually run.

## Recovery note

This package lived in WSL at
`\\wsl.localhost\Ubuntu-22.04\home\rgupta100\swerve_ws\` and was destroyed when
the machine was reset on 2026-07-18. It was reconstructed on 2026-07-29 from
VS Code's local-history store in `C:\Windows.old`, which had preserved **82
saved versions** across the four files:

| File | Versions | First | Last |
|---|---|---|---|
| `src/Testing.cpp` | 50 | 2026-03-06 23:33 | 2026-03-08 22:43 |
| `src/swerve_optimizer.cpp` | 22 | 2026-02-22 13:16 | 2026-03-09 00:21 |
| `CMakeLists.txt` | 8 | 2026-02-22 13:17 | 2026-03-05 22:35 |
| `package.xml` | 2 | 2026-02-22 13:17 | 2026-03-05 22:37 |

The tracked files are the newest version of each. Every intermediate version is kept offline, outside this repository.

Nothing outside those four files survived — no launch files, no URDF, no
`ros2_control` config, and no Gazebo world. So the controllers this node
publishes to (`/swerve_steer_controller`, `/swerve_drive_controller`) are not
defined anywhere in this repo, and it will build but not drive anything until
that side is rebuilt.

## License

Apache-2.0, per `package.xml`.
