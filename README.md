# FLO Assignment — MPPI Path Tracking & Obstacle Avoidance

A ROS 2 Humble autonomous navigation stack for TurtleBot3 using **Model Predictive Path Integral (MPPI)** control with real-time obstacle avoidance in Gazebo.

---

## Prerequisites

- **Ubuntu 22.04** with **ROS 2 Humble**
- **TurtleBot3 packages** (`turtlebot3`, `turtlebot3_simulations`)
- **Gazebo Classic**
- **Python 3.10+** with `numpy`, `scipy`
- A **PS4 controller** (for teleop waypoint collection)

---

## Installation

```bash
cd ~
git clone https://github.com/jivalz/flo_assignment.git
cd flo_assignment
colcon build --symlink-install
source install/setup.bash
export TURTLEBOT3_MODEL=burger
```

---

## Recording Waypoints

Open 4 terminals. Run `source install/setup.bash` and `export TURTLEBOT3_MODEL=burger` in each.

**Terminal 1** — Launch Gazebo + RViz:
```bash
ros2 launch nav sim_bringup.py
```

**Terminal 2** — Start PS4 joystick driver:
```bash
ros2 run joy joy_node
```

**Terminal 3** — Start teleop node:
```bash
ros2 run pypkg teleop
```

**Terminal 4** — Start waypoint collector:
```bash
ros2 run pypkg wp_collector
```

Drive the robot along the desired track using the PS4 controller. Press `Ctrl+C` in Terminal 4 when done to save the waypoints.

---

## Running the MPPI Controller

Open 2 terminals. Run `source install/setup.bash` and `export TURTLEBOT3_MODEL=burger` in each.

**Terminal 1** — Launch Gazebo + RViz:
```bash
ros2 launch nav sim_bringup.py
```

**Terminal 2** — Run the MPPI solver:
```bash
ros2 run pypkg mppi_final
```

To add obstacles during runtime, use the Gazebo toolbar: `Insert` → choose a model (cylinder, box, etc.) → click to place.

---

## Test Case Videos

### Case 1 — Straight Line Tracking
[▶️ Watch on YouTube](https://youtu.be/84PZYuvF5mA)

### Case 2 — Sharp Turns & Curves
[▶️ Watch on YouTube](https://youtu.be/eGq0YXwQRmg)

### Case 3 — Full Track Completion
[▶️ Watch on YouTube](https://youtu.be/5_h0AVIYMv4)

### Case 4 — Static Obstacle Avoidance
[▶️ Watch on YouTube](https://youtu.be/i1TQzU3h8PI)

### Dynamic Obstacle Avoidance
[▶️ Watch on YouTube](https://youtu.be/fjrjhM7Xo4k)
