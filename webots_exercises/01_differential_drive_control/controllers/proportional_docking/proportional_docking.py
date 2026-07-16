"""
e-puck Proportional (P) Waypoint Controller.

This module implements a proportional-error feedback controller to navigate a 
differential-drive e-puck robot toward a central target (0,0) in Webots.
It captures telemetry data and displays a trajectory plot at the end of the trial.
"""

import math
from typing import List, Tuple
import matplotlib.pyplot as plt
from controller import Supervisor


class EPuckController:
    # --- Configuration & Hyperparameters ---
    KP_LINEAR: float = 3.0        # Proportional gain for forward speed
    KP_ANGULAR: float = 4.0       # Proportional gain for rotation
    MAX_SPEED: float = 6.28       # e-puck max motor speed (rad/s)
    STOP_THRESHOLD: float = 0.02  # Stop when within 2 cm of the center
    TARGET: Tuple[float, float] = (0.0, 0.0)

    # --- Physical Robot Parameters ---
    WHEEL_RADIUS: float = 0.0205  # meters
    AXLE_LENGTH: float = 0.052    # meters

    def __init__(self) -> None:
        """Initializes the Webots supervisor and hardware interfaces."""
        self.supervisor = Supervisor()
        self.timestep = int(self.supervisor.getBasicTimeStep())

        # Resolve robot node (Prefers DEF 'EPUCK', fallbacks to self)
        self.robot_node = self.supervisor.getFromDef("EPUCK") or self.supervisor.getSelf()
        if not self.robot_node:
            raise RuntimeError("Failed to resolve the robot node in the scene tree.")

        # Initialize hardware devices
        self._init_motors()

        # Telemetry storage for plotting
        self.trajectory_x: List[float] = []
        self.trajectory_y: List[float] = []

        print("P-Controller Initialized successfully.")

    def _init_motors(self) -> None:
        """Configures left and right motors for velocity control mode."""
        self.left_motor = self.supervisor.getDevice("left wheel motor")
        self.right_motor = self.supervisor.getDevice("right wheel motor")
        
        for motor in (self.left_motor, self.right_motor):
            motor.setPosition(float('inf'))
            motor.setVelocity(0.0)

    @staticmethod
    def normalize_angle(angle: float) -> float:
        """Wraps an angle (in radians) into the interval [-pi, pi]."""
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle

    def get_heading(self, rotation_matrix: List[float]) -> float:
        """Extracts the yaw (heading) angle from a 3x3 rotation matrix."""
        return math.atan2(rotation_matrix[3], rotation_matrix[0])

    def compute_control(self, dx: float, dy: float, yaw: float, distance_error: float) -> Tuple[float, float]:
        """Calculates linear and angular velocities using the P-Control Law."""
        target_angle = math.atan2(dy, dx)
        angle_error = self.normalize_angle(target_angle - yaw)

        # Proportional Steering Control
        angular_velocity = self.KP_ANGULAR * angle_error

        # Turn in place if heading error is severe (> 45 deg) to minimize overshoot
        if abs(angle_error) > math.radians(45):
            linear_velocity = 0.0
        else:
            linear_velocity = self.KP_LINEAR * distance_error

        return linear_velocity, angular_velocity

    def set_wheel_speeds(self, linear_velocity: float, angular_velocity: float) -> None:
        """Converts spatial velocities to individual wheel speeds and applies limits."""
        v_left = (linear_velocity - (angular_velocity * self.AXLE_LENGTH / 2.0)) / self.WHEEL_RADIUS
        v_right = (linear_velocity + (angular_velocity * self.AXLE_LENGTH / 2.0)) / self.WHEEL_RADIUS

        # Clamp velocities to maximum motor limit
        v_left_clamped = max(min(v_left, self.MAX_SPEED), -self.MAX_SPEED)
        v_right_clamped = max(min(v_right, self.MAX_SPEED), -self.MAX_SPEED)

        # Apply velocities to motors
        self.left_motor.setVelocity(v_left_clamped)
        self.right_motor.setVelocity(v_right_clamped)

    def display_plot(self) -> None:
        """Generates and displays your matplotlib trajectory visualization."""
        if not self.trajectory_x:
            print("No trajectory data recorded. Skipping plot.")
            return

        print("Generating Trajectory Plot...")
        plt.figure(figsize=(6, 6))
        plt.plot(self.trajectory_x, self.trajectory_y, label="Robot Path", color='blue', linewidth=2)
        plt.scatter([0], [0], color='red', marker='x', s=100, label="Target (0,0)")
        plt.scatter([self.trajectory_x[0]], [self.trajectory_y[0]], color='green', marker='o', label="Start Position")
        plt.title("e-puck P-Controller Trajectory to Center")
        plt.xlabel("X Position (m)")
        plt.ylabel("Y Position (m)")
        plt.grid(True)
        plt.axis('equal')
        plt.legend()
        plt.savefig("epuck_trajectory.png")
        plt.show()

    def run(self) -> None:
        """Main execution flow for tracking state and computing motor commands."""
        while self.supervisor.step(self.timestep) != -1:
            # 1. State Estimation
            pos = self.robot_node.getPosition()
            rot = self.robot_node.getOrientation()
            
            x_curr, y_curr = pos[0], pos[1]
            self.trajectory_x.append(x_curr)
            self.trajectory_y.append(y_curr)

            yaw = self.get_heading(rot)

            # 2. Distance Error Calculation
            target_x, target_y = self.TARGET
            dx = target_x - x_curr
            dy = target_y - y_curr
            distance_error = math.sqrt(dx**2 + dy**2)

            # 3. Termination Condition Check
            if distance_error < self.STOP_THRESHOLD:
                print("Target Reached! Stopping motors...")
                self.left_motor.setVelocity(0.0)
                self.right_motor.setVelocity(0.0)
                break

            # 4. Control Loop Calculation and Application
            linear_v, angular_v = self.compute_control(dx, dy, yaw, distance_error)
            self.set_wheel_speeds(linear_v, angular_v)


if __name__ == "__main__":
    controller = EPuckController()
    try:
        controller.run()
    finally:
        controller.display_plot()