from controller import Robot
import math
import matplotlib.pyplot as plt

# Initialize robot
robot = Robot()
timestep = int(robot.getBasicTimeStep())

left_motor = robot.getDevice("left wheel motor")
right_motor = robot.getDevice("right wheel motor")
left_motor.setPosition(float('inf'))
right_motor.setPosition(float('inf'))
left_motor.setVelocity(0.0)
right_motor.setVelocity(0.0)


gps = robot.getDevice("gps")
compass = robot.getDevice("compass")
left_pos_sensor = robot.getDevice("left wheel sensor")
right_pos_sensor = robot.getDevice("right wheel sensor")

gps.enable(timestep)
compass.enable(timestep)
left_pos_sensor.enable(timestep)
right_pos_sensor.enable(timestep)

# Robot parameters
wheel_radius = 0.0205 
wheel_distance = 0.052 


time_data = []
gps_data = []
compass_data = []
odom_data = []

# Initial odometry state
odom_x, odom_y, odom_theta = 0.0, 0.0, 0.0
prev_left_pos = 0.0
prev_right_pos = 0.0


sim_time = 0
max_time = 20.0 

def normalize(alpha):
    while alpha > math.pi:
        alpha -= 2 * math.pi
    while alpha < -math.pi:
        alpha += 2 * math.pi
    return alpha

while robot.step(timestep) != -1 and sim_time <= max_time:
    sim_time = robot.getTime()
    time_data.append(sim_time)
    
    scenario = 3  #1, 2, 3
    
    if scenario == 1:
        right_speed = 1.0  # rad/s
        left_speed = 2.0  # rad/s
    elif scenario == 2:
        right_speed = math.pi  # π rad/s
        left_speed = -math.pi  # -π rad/s
    elif scenario == 3:
        right_speed = 5 * math.sin(sim_time)
        left_speed = -5 * math.cos(sim_time)
    
    left_motor.setVelocity(left_speed)
    right_motor.setVelocity(right_speed)
    
    # Get sensor data
    # GPS data
    gps_values = gps.getValues()
    gps_data.append((gps_values[0], gps_values[1]))  # x, y
    
    # Compass data (orientation)
    compass_values = compass.getValues()
    compass_theta = math.atan2(compass_values[0], compass_values[1])
    compass_data.append(compass_theta)
    
    # Odometry calculation
    left_pos = left_pos_sensor.getValue()
    right_pos = right_pos_sensor.getValue()
    
    # differences
    delta_left = left_pos - prev_left_pos
    delta_right = right_pos - prev_right_pos
    
    # Update odometry
    # delta_distance = (delta_left + delta_right) * wheel_radius / 2.0
    # delta_theta = (delta_right - delta_left) * wheel_radius / wheel_distance
    # odom_x += delta_distance * math.cos(odom_theta + (delta_theta / 2.0))
    # odom_y += delta_distance * math.sin(odom_theta + (delta_theta / 2.0))
    # odom_theta += delta_theta
    # odom_theta = normalize(odom_theta)
    v_l = delta_left * wheel_radius / (timestep / 1000.0)
    v_r = delta_right * wheel_radius / (timestep / 1000.0)

    v = (v_r + v_l) / 2.0
    omega = (v_r - v_l) / wheel_distance

    odom_x += v * math.cos(odom_theta) * (timestep / 1000.0)
    odom_y += v * math.sin(odom_theta) * (timestep / 1000.0)
    odom_theta += omega * (timestep / 1000.0)
    odom_theta = normalize(odom_theta)

    odom_data.append((odom_x, odom_y, odom_theta))
    
    # Store current positions for next iteration
    prev_left_pos = left_pos
    prev_right_pos = right_pos


# Plot
plt.figure(figsize=(12, 6))

# GPS trajectory
gps_x, gps_y = zip(*gps_data)
plt.subplot(1, 2, 1)
plt.plot(gps_x, gps_y, label='GPS')
plt.title('X-Y Position (GPS)')
plt.xlabel('X position (m)')
plt.ylabel('Y position (m)')
plt.grid(True)

# Odometry trajectory
odom_x, odom_y, _ = zip(*odom_data)
plt.plot(odom_x, odom_y, label='Odometry')
plt.legend()

# Plot orientation over time
plt.subplot(1, 2, 2)
plt.plot(time_data, compass_data, label='Compass')
plt.plot(time_data, [theta for (_, _, theta) in odom_data], label='Odometry')
plt.title('Orientation vs Time')
plt.xlabel('Time (s)')
plt.ylabel('Orientation (rad)')
plt.legend()
plt.grid(True)

plt.tight_layout()
plt.savefig('position_tracking_results.png')
plt.show()