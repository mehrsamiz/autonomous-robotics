"""
Open-Loop Velocity Profiling for Differential Drive Robot.
Explores the spatial trajectory effects of static and time-varying wheel inputs.
"""
import math
import matplotlib.pyplot as plt
from controller import Robot

# Initialize robot and devices
robot = Robot()
timestep = int(robot.getBasicTimeStep())

left_motor = robot.getDevice('left wheel motor')
right_motor = robot.getDevice('right wheel motor')
left_motor.setPosition(float('inf'))
right_motor.setPosition(float('inf'))

gps = robot.getDevice('gps')
gps.enable(timestep)

# Physical parameters
WHEEL_RADIUS = 0.0205  # m (e-puck standard)
path = []

# Toggle to evaluate different actuation profiles
RUN_CASE_1 = True  # Set to False to evaluate dynamic harmonic velocities

while robot.step(timestep) != -1:
    time_now = robot.getTime()

    if RUN_CASE_1:
        # Case 1: Pure rotational moment (opposite static angular velocities)
        phi1 = 1.0   # Right wheel (rad/s)
        phi2 = -1.0  # Left wheel (rad/s)
    else:
        # Case 2: Time-varying asymmetric input
        phi1 = math.cos(2 * time_now)
        phi2 = math.sin(-time_now)
    
    # Set motor velocities directly (rad/s)
    right_motor.setVelocity(phi1)
    left_motor.setVelocity(phi2)

    # Capture 2D coordinates
    position = gps.getValues()
    path.append((position[0], position[1])) 

    # Break simulation after 500 steps to isolate path data
    if time_now > 50.0: 
        break

# Plot trajectory
x_coords, y_coords = zip(*path)
plt.figure(figsize=(6, 6))
plt.plot(x_coords, y_coords, label='Robot Path', color='royalblue')
plt.xlabel('X Position (m)')
plt.ylabel('Y Position (m)')
plt.title('Open-Loop Actuation Profile')
plt.grid(True)
plt.legend()
plt.axis('equal')
plt.show()