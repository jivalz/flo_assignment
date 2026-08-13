# FLO Assignment — MPPI Path Tracking & Obstacle Avoidance

A ROS 2 Humble autonomous navigation stack for TurtleBot3 using **Model Predictive Path Integral (MPPI)** control with real-time obstacle avoidance in Gazebo.

---

## Prerequisites

- **Ubuntu 22.04** with **ROS 2 Humble**
- **TurtleBot3 packages** (`turtlebot3`, `turtlebot3_simulations`)
- **Gazebo Classic**
- **Python 3.10+** with `numpy`, `scipy`
- A **PS4 controller** (optional, for teleop waypoint collection)

---

## Installation

```bash
# Clone the repository
cd ~
git clone https://github.com/<your-username>/flo_assignment.git
cd flo_assignment

# Build the workspace
colcon build --symlink-install
source install/setup.bash

# Set TurtleBot3 model (add to ~/.bashrc for persistence)
export TURTLEBOT3_MODEL=burger
```

> **Note:** Run `source install/setup.bash` in every new terminal, or add it to your `~/.bashrc`.

---

## Quick Start

### Step 1 — Launch Gazebo + RViz

Open **Terminal 1**:

```bash
export TURTLEBOT3_MODEL=burger
ros2 launch nav sim_bringup.py
```

This launches:
- Gazebo with an empty world and the TurtleBot3 Burger
- RViz2 with pre-configured visualization for MPPI rollouts, optimal path, and reference path

> **Tip:** To add obstacles, use the Gazebo toolbar: `Insert` → choose a model (cylinder, box, etc.) → click to place it in the world.

---

### Step 2 — Record Waypoints (First Time Only)

If you need to create a new waypoint track, use the **PS4 Teleop** + **Waypoint Collector** workflow.

#### 2a. Start the PS4 Teleop Node

Open **Terminal 2**:

```bash
ros2 run pypkg teleop
```

**PS4 Controller Mapping:**
| Button / Stick | Action |
|---|---|
| Left Stick Y-axis | Forward / Backward |
| Right Stick X-axis | Turn Left / Right |
| L1 / R1 | Decrease / Increase linear speed |
| L2 / R2 | Decrease / Increase angular speed |

> **Note:** If the PS4 controller is not connected, the node will wait for input. Make sure the controller is paired via Bluetooth or USB before launching.

#### 2b. Start the Waypoint Collector

Open **Terminal 3**:

```bash
ros2 run pypkg wp_collector
```

This will automatically:
1. Create a new `waypointN.csv` file in `src/nav/waypoints/` (auto-incrementing filename)
2. Record the robot's `(x, y)` position from `/odom` every time it moves more than `0.1m`
3. Save the waypoints on shutdown (`Ctrl+C`)

**Custom filename:**
```bash
ros2 run pypkg wp_collector --ros-args -p file_name:=my_track
```

Now drive the robot around the track with your PS4 controller. When done, press `Ctrl+C` in the waypoint collector terminal to save.

---

### Step 3 — Run the MPPI Controller

Open **Terminal 2** (or a new terminal):

```bash
ros2 run pypkg mppi_final
```

The robot will:
1. Load the waypoint CSV file
2. Generate a smooth cubic spline reference path
3. Begin autonomous path tracking using the MPPI solver
4. Dynamically avoid obstacles detected by the LiDAR

**To use a specific waypoint file:**
```bash
ros2 run pypkg mppi_final --ros-args -p waypoint_file:=/path/to/your/waypoints.csv
```

---

## Waypoint Files

Pre-recorded tracks are stored in `src/nav/waypoints/`:

| File | Description |
|---|---|
| `waypoint1.csv` | Basic straight-line test |
| `waypoint2.csv` | Simple curved path |
| `waypoint3.csv` | Complex track with sharp turns |
| `waypoint4.csv` | Long track with multiple segments |
| `waypoint5.csv` | Full obstacle avoidance test track |

---

## RViz Visualization

| Topic | Color | Description |
|---|---|---|
| `/mppi/ref_path` | Blue | Reference path (cubic spline) |
| `/mppi/optimal_path` | Green | Best trajectory from current solve |
| `/mppi/rollouts` | Light Green (transparent) | Fan of 40 candidate rollout trajectories |
| `/cmd_vel` | — | Velocity commands sent to the robot |

---

## Test Case Videos

### Case 1 — Straight Line Tracking
[▶️ Watch on YouTube](https://youtu.be/84PZYuvF5mA)

---

### Case 2 — Sharp Turns & Curves
[▶️ Watch on YouTube](https://youtu.be/eGq0YXwQRmg)

---

### Case 3 — Full Track Completion
[▶️ Watch on YouTube](https://youtu.be/5_h0AVIYMv4)

---

### Case 4 — Static Obstacle Avoidance
[▶️ Watch on YouTube](https://youtu.be/i1TQzU3h8PI)

---

### Dynamic Obstacle Avoidance
[▶️ Watch on YouTube](https://youtu.be/fjrjhM7Xo4k)
