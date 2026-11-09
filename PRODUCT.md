# Product

<!-- impeccable:product-schema 1 -->

## Platform

web

## Stack

Static HTML/CSS/JS with no build step, served by a ROS2 Python node (`panel_server`) inside the `minisumo_ros` package. The node also bridges ROS2 topics to the browser over a WebSocket built on the Python standard library. No rosbridge, so nothing has to be installed with sudo in the VM. Every library and font is vendored inside the package, so the panel works fully offline, including on SoftAP or without internet at the defense. The stack was delegated: chosen by the assistant for these constraints, and the user approved offline vendoring.

## Users

- **Primary user:** the student team that built CAR-LOSS. They drive and tune the robot in the lab, and present it to the professor and jury at the course defense.
- **What matters most (user's words):** the panel should impress, be intuitive and easy to use, and feel like a professional, interactive tool with several capabilities.

## Product Purpose

A single live control console for the CAR-LOSS teleoperated minisumo robot. It replaces juggling RViz and the matplotlib signal plotter. It shows where the robot is and how it moves on the dojo, the health of the robot (connection, battery, control loop), and what the glove is commanding. It shows how the signals look raw and after the Kalman filters. It also lets the operator act on the robot: stop, reset, recalibrate, self-test and tune gains live.

Success: in a demo, the professor understands the whole system at a glance, and the team can run tests and tuning from it without touching a terminal.

## Positioning

It is built around this specific robot and its real data path: ESP32 firmware → UDP → ROS2 → Kalman nodes. It is not a generic robot dashboard. The 3D model is the team's own SolidWorks assembly. The numbers are the firmware's real telemetry columns. The controls are the firmware's own UDP commands.

## Operating Context

- **Where it runs:** in the lab and during the defense. The operator wears the glove (ESP32 + MPU-6050) while the robot runs on a 77 cm minisumo dojo.
- **Software:** ROS2 Lyrical runs in an Ubuntu 26.04 VirtualBox VM (4 cores, about 3.4 GB RAM, Firefox). The browser may be on the VM, on the Windows host, or on a phone on the same network.
- **Network:** the lab WiFi is intermittent, and a SoftAP mode is planned for later.
- **Existing tools stay as fallbacks:**
  - RViz (`visualizacion.launch.py`)
  - `signal_plotter`
  - CSV logging (`data_logger`)

## Capabilities and Constraints

- **Robot data (telemetry at ~200 samples/s):**
  - IMU raw and filtered
  - wheel speeds, measured and commanded (rad/s)
  - duty cycle of each motor
  - encoder ticks
  - odometry: x, y, theta, plus theta from gyro and from encoders
  - control mode (0 stop, 1 velocity PID, 2 open loop)
  - battery 2S voltage, current and power (INA219)
- **Kalman topics:**
  - `/imu/kalman`
  - `/wheels/kalman` [ω, α]
  - `/glove/imu/kalman`
  - `/glove/angles_kalman`
- **Glove:** hand pitch and roll (raw accelerometer and filtered), glove IMU, and the resulting `/cmd_vel`.
- **Firmware commands through `/robot_cmd`:**

  | Command | Action |
  |---|---|
  | `S` | stop |
  | `R` | reset odometry |
  | `C` | recalibrate gyro (robot still, 2 s) |
  | `T` | self-test, report on serial |
  | `G,kp,ki,kd[,kff]` | wheel PID |
  | `F,dmin_l,dmin_r[,trim]` | dead zone |
  | `Y,kp,ki[,kb]` | heading control |
  | `M,dl,dr` | open-loop duty |

  Wheel commands go through `/wheel_cmd` and velocity through `/cmd_vel`.
- **Robot facts:**
  - wheel radius 16.2 mm (effective), wheel base 79 mm
  - wheel max ≈13.5 rad/s, about 0.2 m/s; use ≤12 rad/s
  - battery low below 7.0 V, critical below 6.6 V
- **Constraints:**
  - VM memory is tight, so the page must stay light and be throttled.
  - The HUD must work offline.
  - Gains set live are RAM-only on the ESP32 and lost on reboot.

## Brand Commitments

- **Project name:** CAR-LOSS.
- **Robot colors (from the CAD):** black body, yellow wheels, metallic motors.
- No logo has been provided.

## Evidence on Hand

- **3D model:** `ros2/minisumo_ros/meshes/*.stl`, converted from the SolidWorks assembly with `herramientas/cad2rviz.py`.
- **Live telemetry:** all the topics listed above.
- **Measured results that may be quoted:**
  - Heading recovery: after a push of +16 to +27°, the robot returns to within 3° in about 0.2–0.3 s.
  - Kalman noise reduction on synthetic tests: accelerometer σ 0.51 → 0.13 m/s², wheel speed σ 0.37 → 0.10 rad/s, hand angle error 2.0 → 0.22°.
- **Absences:** there are no team photos, no logo and no member names. Do not fabricate them.

## Product Principles

1. Real data first: every number on screen comes from a live topic, and stale data is shown as stale, never as zero.
2. Safety is always reachable: stop is one tap or one key from anywhere.
3. Explain the system: a newcomer must be able to follow the chain glove → command → wheels → motion → filters without the team narrating.
4. Light on the machine: the page must never starve the VM that runs ROS2.
5. Spanish UI, with the units the team already uses (rad/s, cm/s, deg, V, mA).
