"""
Localization Controller for e-puck Robot (Webots)
Compares Dead Reckoning (Odometry) against Ground Truth Sensors (GPS & Compass).
"""

import os
import math
import matplotlib.pyplot as plt
from controller import Robot

# =========================================================================
# CONFIGURATION: Change this to 1, 2, or 3 to test different motion profiles
SCENARIO = 2  
# =========================================================================

class DifferentialDriveOdometer:
    """
    Calculates dead reckoning (odometry) for a differential drive robot
    using 2nd-order Runge-Kutta (midpoint) integration.
    """
    def __init__(self, wheel_radius, wheel_distance):
        self.R = wheel_radius
        self.L = wheel_distance
        
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        
        self.prev_left_pos = 0.0
        self.prev_right_pos = 0.0

    def update(self, left_pos, right_pos):
        # Calculate encoder deltas
        delta_left = left_pos - self.prev_left_pos
        delta_right = right_pos - self.prev_right_pos
        
        # Save positions for the next iteration
        self.prev_left_pos = left_pos
        self.prev_right_pos = right_pos
        
        # Convert to linear displacements
        ds_left = delta_left * self.R
        ds_right = delta_right * self.R
        
        # Compute trajectory step
        ds = (ds_left + ds_right) / 2.0
        d_theta = (ds_right - ds_left) / self.L
        
        # Midpoint Integration (RK2 approximation)
        mid_theta = self.theta + (d_theta / 2.0)
        self.x += ds * math.cos(mid_theta)
        self.y += ds * math.sin(mid_theta)
        self.theta = self.normalize(self.theta + d_theta)
        
        return self.x, self.y, self.theta

    @staticmethod
    def normalize(angle):
        """Keep heading angle bounded between -pi and pi."""
        return (angle + math.pi) % (2 * math.pi) - math.pi


def main():
    # Instantiate the Robot ONLY ONCE
    robot = Robot()
    timestep = int(robot.getBasicTimeStep())
    
    print(f"Initializing localization simulation for Scenario {SCENARIO}...")
    
    # Initialize Actuators
    left_motor = robot.getDevice("left wheel motor")
    right_motor = robot.getDevice("right wheel motor")
    left_motor.setPosition(float('inf'))
    right_motor.setPosition(float('inf'))
    left_motor.setVelocity(0.0)
    right_motor.setVelocity(0.0)
    
    # Initialize Sensors
    gps = robot.getDevice("gps")
    compass = robot.getDevice("compass")
    left_pos_sensor = robot.getDevice("left wheel sensor")
    right_pos_sensor = robot.getDevice("right wheel sensor")
    
    gps.enable(timestep)
    compass.enable(timestep)
    left_pos_sensor.enable(timestep)
    right_pos_sensor.enable(timestep)
    
    # e-puck Kinematic Parameters
    R_wheel = 0.0205 
    L_wheelbase = 0.052 
    
    # Instantiate Odometer
    odometer = DifferentialDriveOdometer(R_wheel, L_wheelbase)
    
    # Data Loggers
    time_data = []
    gps_data = []
    compass_data = []
    odom_data = []
    
    sim_time = 0.0
    max_time = 20.0 
    
    # Main simulation loop
    while robot.step(timestep) != -1 and sim_time <= max_time:
        sim_time = robot.getTime()
        time_data.append(sim_time)
        
        # Apply speeds based on the configured scenario
        if SCENARIO == 1:
            left_speed = 2.0
            right_speed = 1.0
        elif SCENARIO == 2:
            left_speed = -math.pi
            right_speed = math.pi
        elif SCENARIO == 3:
            left_speed = -5 * math.cos(sim_time)
            right_speed = 5 * math.sin(sim_time)
        else:
            print(f"Error: Unknown scenario {SCENARIO}")
            return
            
        left_motor.setVelocity(left_speed)
        right_motor.setVelocity(right_speed)
        
        # Read absolute sensors
        gps_vals = gps.getValues()
        gps_data.append((gps_vals[0], gps_vals[1]))
        
        compass_vals = compass.getValues()
        compass_theta = math.atan2(compass_vals[0], compass_vals[1])
        compass_data.append(compass_theta)
        
        # Update our dead reckoning estimate
        left_ticks = left_pos_sensor.getValue()
        right_ticks = right_pos_sensor.getValue()
        odom_pose = odometer.update(left_ticks, right_ticks)
        odom_data.append(odom_pose)
        
    # Generate and save visual results once simulation finishes
    print(f"Simulation completed. Saving plots for Scenario {SCENARIO}...")
    plot_results(SCENARIO, time_data, gps_data, compass_data, odom_data)


def plot_results(scenario, time_data, gps_data, compass_data, odom_data):
    plt.figure(figsize=(12, 5))
    
    gps_x, gps_y = zip(*gps_data)
    odom_x, odom_y, odom_theta = zip(*odom_data)
    
    # Plot 1: Trajectory
    plt.subplot(1, 2, 1)
    plt.plot(gps_x, gps_y, 'g-', label='GPS Ground Truth', linewidth=2)
    plt.plot(odom_x, odom_y, 'r--', label='Odometry (RK2 Estimate)', linewidth=1.5)
    plt.title(f'Trajectory Comparison (Scenario {scenario})')
    plt.xlabel('X Position (m)')
    plt.ylabel('Y Position (m)')
    plt.grid(True, linestyle=':')
    plt.legend()
    
    # Plot 2: Orientation
    plt.subplot(1, 2, 2)
    plt.plot(time_data, compass_data, 'g-', label='Compass Ground Truth', linewidth=2)
    plt.plot(time_data, odom_theta, 'r--', label='Odometry (RK2 Estimate)', linewidth=1.5)
    plt.title(f'Heading Comparison (Scenario {scenario})')
    plt.xlabel('Time (s)')
    plt.ylabel('Orientation (rad)')
    plt.grid(True, linestyle=':')
    plt.legend()
    
    plt.tight_layout()
    
    # Save directly to your workspace data folder
    output_dir = '../../data'
    os.makedirs(output_dir, exist_ok=True)
    plt.savefig(os.path.join(output_dir, f'scenario{scenario}_results.png'), dpi=300)
    plt.close()


if __name__ == '__main__':
    main()