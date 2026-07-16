"""
Inverse Kinematics Controller.
Maps target linear (v) and angular (w) spatial velocities to wheel actuation speeds.
"""
import numpy as np
import matplotlib.pyplot as plt
from controller import Robot

# Physical e-puck properties
WHEEL_RADIUS = 0.0205  # m
WHEELBASE = 0.053     # L (m)

# Simulation run configurations
TEST_CASES = [
    {"v": 0.04, "omega": 0.2},   # Test Case 1
    {"v": 0.01, "omega": -0.3}   # Test Case 2
]

robot = Robot()
timestep = int(robot.getBasicTimeStep())

left_motor = robot.getDevice('left wheel motor')
right_motor = robot.getDevice('right wheel motor')
left_motor.setPosition(float('inf'))
right_motor.setPosition(float('inf'))

gps = robot.getDevice('gps')
gps.enable(timestep)

# Select test configuration index
active_case = TEST_CASES[0]
v = active_case["v"]
omega = active_case["omega"]

# Inverse Kinematics transformation
omega_l = (2.0 * v - omega * WHEELBASE) / (2.0 * WHEEL_RADIUS)
omega_r = (2.0 * v + omega * WHEELBASE) / (2.0 * WHEEL_RADIUS)

# Set computed wheel rates
left_motor.setVelocity(omega_l)
right_motor.setVelocity(omega_r)

x_path, y_path = [], []
simulation_duration = 60.0  # Run test for 60 seconds

while robot.step(timestep) != -1:
    elapsed_time = robot.getTime()
    if elapsed_time > simulation_duration:
        break

    gps_values = gps.getValues()
    x_path.append(gps_values[0])
    y_path.append(gps_values[1])

# Plot resulting spatial trajectory
plt.figure(figsize=(6, 6))
plt.plot(x_path, y_path, label=f'v={v} m/s, ω={omega} rad/s', color='forestgreen')
plt.xlabel('X (m)')
plt.ylabel('Y (m)')
plt.title('Inverse Kinematics Trajectory Output')
plt.grid(True)
plt.legend()
plt.axis('equal')
plt.show()