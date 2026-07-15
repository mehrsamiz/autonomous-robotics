# Autonomous Mobile Robotics Portfolio

A comprehensive collection of autonomous navigation, state estimation, and path-planning algorithms implemented in the Webots simulator using Python.

---

## 🚀 Featured Project: UAV-UGV Collaborative Mapping & Navigation
A heterogeneous multi-agent system solving a complex search-and-navigate problem in a multi-room apartment.

* **Phase 1 (UAV Exploration):** An autonomous drone (`Mavic 2 Pro`) maps the apartment, detects QR codes on doors using visual sensors, and records target coordinates.
* **Phase 2 (UGV Navigation):** A `TurtleBot3 Burger` is "kidnapped" (randomly spawned) without GPS/Compass data. It uses sensor fusion (Kalman Filtering/Odometry) to estimate its position and autonomously navigates to the target door identified by the UAV.

---

## 📂 Core Robotics Labs

### 1. Go-To-Goal Kinematic Controller (Closed-Loop)
* **Objective:** Drive an e-puck robot to a target coordinate $(0, 0)$ using dynamic error feedback.
* **Control Law:** Implements proportional control with dynamic gain scheduling to switch between forward and reverse maneuvers based on heading error:
  $$\text{If } \theta_e \in [-\frac{\pi}{2}, \frac{\pi}{2}] \implies K_{p,\text{dist}} = 0.1 \quad \text{Else} \implies K_{p,\text{dist}} = -0.1$$
* **Visuals:** ![Trajectory Error Plot](assets/go_to_goal_plot.png)

### 2. Reactive Navigation: Bug Algorithms & Wall Following
* **Objective:** Navigate an e-puck robot past obstacles using Bug 0, Bug 1, and Bug 2 algorithms, seamlessly transitioning to a right-wall following behavior.

### 3. Feature Extraction (IEPF & Split-and-Merge)
* **Objective:** Process raw `DistanceSensor` data from `hall.wbt` to reconstruct a line-segment map of the environment using Iterative End-Point Fit (IEPF).

### 4. Kinematics & Odometry Drift Analysis
* **Objective:** Model the differential kinematics of the e-puck and compare raw wheel odometry integration against ground-truth GPS/Compass data.