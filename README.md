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

## Layout

```
src/my_swerve_control/
  src/swerve_optimizer.cpp   ROS 2 node — publishes to the steer/drive controllers
  src/Testing.cpp            standalone algorithm sandbox, no ROS dependency
  CMakeLists.txt
  package.xml
```

`swerve_optimizer.cpp` runs a 50 Hz timer and publishes `Float64MultiArray` to
`/swerve_steer_controller/commands` and `/swerve_drive_controller/commands`.

`Testing.cpp` is where the maths was worked out — it compiles with plain `g++`
and prints the six (angle, velocity) pairs for a given twist, which made it
possible to check the solver by hand before involving Gazebo.

## Build

```bash
cd swerve_ws
colcon build --packages-select my_swerve_control
source install/setup.bash
ros2 run my_swerve_control swerve_optimizer
```

`Testing.cpp` standalone:

```bash
g++ src/my_swerve_control/src/Testing.cpp -o testing && ./testing
```

## Status — what is and isn't finished

Read this before assuming the node is production-ready.

- **The twist is hardcoded.** `kinematics_loop()` uses `Vx = 0.2, Vy = 0.2,
  ω = 0.0`. It does not subscribe to `/cmd_vel` yet — that's the next change.
- **Module-angle optimisation is written but commented out** in `Testing.cpp`.
  `optimize_module()` wraps the steering delta to [−180°, 180°] and, when the
  required turn exceeds 90°, flips the module to `θ ± 180°` and negates the
  drive velocity — so a module never swings more than a quarter turn to reach a
  heading. Not yet ported into the ROS node.
- **Singularity handling is written but commented out.** `singularity()` holds
  the previous steering angle when commanded speed falls below 0.001 rather
  than letting `atan2(0, 0)` snap modules to zero — this is the near-ICR /
  stopped case.
- **Speed desaturation is written but commented out.** `normalise_speeds()`
  scales all six velocities down proportionally when any module exceeds the
  motor limit, preserving the direction of travel.
- **No efficiency benchmark exists in this repo.** There is no baseline
  comparison and no measurement harness here. Any efficiency figure quoted
  elsewhere is not reproducible from this code as it stands.
- **Units are mixed by design and it's a trap.** `Testing.cpp` outputs degrees
  (`* 180 / M_PI`); `swerve_optimizer.cpp` publishes raw radians from `atan2`.
  Don't copy a constant from one to the other.

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

The files here are the newest version of each. The intermediate versions are
kept offline and are not part of this repository.

Nothing outside those four files survived — no launch files, no URDF, no
`ros2_control` config, and no Gazebo world. So the controllers this node
publishes to (`/swerve_steer_controller`, `/swerve_drive_controller`) are not
defined anywhere in this repo, and it will build but not drive anything until
that side is rebuilt.

## License

Apache-2.0, per `package.xml`.
