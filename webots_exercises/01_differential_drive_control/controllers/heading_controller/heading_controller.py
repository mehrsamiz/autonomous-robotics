"""
Closed-Loop Angular Heading Controller.
Uses a Compass sensor with normalize-to-range logic to steer to target headings.
"""
import math
import matplotlib.pyplot as plt
from controller import Robot

robot = Robot()
timestep = int(robot.getBasicTimeStep())

left_motor = robot.getDevice('left wheel motor')
right_motor = robot.getDevice('right wheel motor')
left_motor.setPosition(float('inf'))
right_motor.setPosition(float('inf'))

compass = robot.getDevice('compass')
compass.enable(timestep)

# Controller parameters
DESTINATION_ANGLE = math.radians(0)  # Target heading (radians)
ANGLE_TOLERANCE = math.radians(0.1)  # Stop condition threshold
MAX_SPEED = 4.0                      # Max motor angular velocity (rad/s)
MIN_SPEED = 0.1                      # Avoid actuator dead-zone stalling
KP = 5.0                             # Proportional gain

heading_over_time = []

def normalize_angle(angle):
    """Normalize angles to the [-pi, pi] interval."""
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle

while robot.step(timestep) != -1:    
    compass_values = compass.getValues()
    
    # Calculate current heading from 2D compass vectors
    current_angle = math.atan2(compass_values[1], compass_values[0])
    
    # Compute minimal angular error
    error = normalize_angle(DESTINATION_ANGLE - current_angle)
    heading_over_time.append((robot.getTime(), math.degrees(current_angle)))

    # Evaluate arrival
    if abs(error) < ANGLE_TOLERANCE:
        left_motor.setVelocity(0.0)
        right_motor.setVelocity(0.0)
        print("[INFO] Target heading locked successfully.")
        break

    # Compute proportional angular speed
    rotation_speed = max(MIN_SPEED, min(MAX_SPEED, KP * abs(error)))

    # Apply differential rotational drive commands
    if error > 0:
        left_motor.setVelocity(rotation_speed)
        right_motor.setVelocity(-rotation_speed)
    else:
        left_motor.setVelocity(-rotation_speed)
        right_motor.setVelocity(rotation_speed)

# Plot response performance
times, headings = zip(*heading_over_time)
plt.figure(figsize=(8, 4))
plt.plot(times, headings, color='crimson', linewidth=2)
plt.axhline(y=math.degrees(DESTINATION_ANGLE), color='black', linestyle='--', label='Target')
plt.xlabel('Time (s)')
plt.ylabel('Heading Angle (deg)')
plt.title('Heading Control Convergence over Time')
plt.grid(True)
plt.legend()
plt.show()