"""
Webots Controller: Bug Navigation & Wall-Following
Author: Mehrsa Samiezadeh
Description: Implements Bug 0, Bug 1, and Bug 2 path planning on an e-puck robot,
             with seamless transition to right-wall following.
"""

import csv
import math
from controller import Robot, GPS, Compass, DistanceSensor

# --- CONFIGURATION & CONSTANTS ---
TIME_STEP = 16
MAX_SPEED = 6.28
GOAL = [0.0, 4.7]                 # Primary goal coordinate (doorway)
CABINET_GOAL = [-3.2, 4.1]        # Final cabinet coordinate
M_LINE = 0.0                      # X-coordinate of the M-line (x = 0)

# --- GLOBAL STATE VARIABLES ---
STATE = 0
mode = "bug0"                     # Selected execution mode: "bug0", "bug1", "bug2"
path_points = []                  # Full coordinate log
path_to_door = []                 # Coordinate log for phase 1 (to doorway)
path_wall_following = []          # Coordinate log for phase 2 (to cabinet)

# --- INITIALIZATION ---
robot = Robot()

# Sensors
gps = robot.getDevice('gps')
gps.enable(TIME_STEP)

compass = robot.getDevice('compass')
compass.enable(TIME_STEP)

# Initialize 6 IR distance sensors
ds = []
for i in range(6):
    sensor = robot.getDevice(f'IR{i}')
    sensor.enable(TIME_STEP)
    ds.append(sensor)

# Actuators
left_motor = robot.getDevice('left wheel motor')
right_motor = robot.getDevice('right wheel motor')
left_motor.setPosition(float('inf'))
right_motor.setPosition(float('inf'))
left_motor.setVelocity(0.0)
right_motor.setVelocity(0.0)


# --- MATHEMATICAL UTILITIES ---

def calculate_path_length(path):
    """Calculates cumulative Euclidean distance traversed along a path."""
    length = 0.0
    for i in range(1, len(path)):
        length += math.dist(path[i-1], path[i])
    return length


def calculate_deviation(path, start, goal):
    """
    Calculates the maximum perpendicular deviation of the robot's trajectory
    from the straight-line path connecting start and goal.
    """
    x1, y1 = start
    x2, y2 = goal
    max_dev = 0.0
    
    for x0, y0 in path:
        num = abs((y2 - y1) * x0 - (x2 - x1) * y0 + x2 * y1 - y2 * x1)
        den = math.hypot(y2 - y1, x2 - x1)
        # Avoid division by zero if start and goal are identical
        dev = num / den if den != 0 else 0.0
        if dev > max_dev:
            max_dev = dev
    return max_dev


def distance(a, b):
    """Calculates Euclidean distance between two 2D points."""
    return math.hypot(a[0] - b[0], a[1] - b[1])


def angle_diff(a, b):
    """Calculates signed angular difference mapped to [-180, 180] degrees."""
    return (a - b + 180) % 360 - 180


def get_bearing():
    """Computes the current robot heading in degrees [0, 360)."""
    compass_val = compass.getValues() 
    angle_rad = math.atan2(compass_val[0], compass_val[1])
    angle_deg = (math.degrees(angle_rad) + 90) % 360  
    return angle_deg


def is_on_M_line(x_current, y_current, threshold=0.06):
    """Checks if the robot is within a lateral threshold of the M-line."""
    return abs(x_current - M_LINE) < threshold 


def calculate_theta(x1, y1, x0, y0):
    """Calculates targeted heading angle toward target from current position."""
    # Prevent division by zero
    if x1 == x0:
        angle_rad = math.pi / 2 if y1 > y0 else -math.pi / 2
    else:
        angle_rad = math.atan((y1 - y0) / (x1 - x0)) 
        
    angle_deg = (math.degrees(angle_rad) % 360) + 90 
    if angle_deg > 358:
        angle_deg = 180
    return angle_deg


# --- MOTION CONTROL FUNCTIONS ---

def stop():
    """Stops both wheel motors immediately."""
    left_motor.setVelocity(0.0)
    right_motor.setVelocity(0.0)


def turn_to(target):
    """Executes closed-loop proportional turning toward target coordinate."""
    while robot.step(TIME_STEP) != -1:
        pos = gps.getValues()
        angle_to_goal = calculate_theta(target[0], target[1], pos[0], pos[1])
        bearing = get_bearing()
        error = angle_diff(angle_to_goal, bearing)
        
        if abs(error) < 5.0:
            break
            
        # Rotate in place based on sign of heading error
        left_speed = -0.5 if error > 0 else 0.5
        right_speed = 0.5 if error > 0 else -0.5
        
        left_motor.setVelocity(left_speed * MAX_SPEED)
        right_motor.setVelocity(right_speed * MAX_SPEED)


def go_straight():
    """Moves the robot straight while verifying path alignment and obstacle presence."""
    right_front = ds[3].getValue()
    pos = gps.getValues()
    
    path_points.append([pos[0], pos[1]])
    angle_to_goal = calculate_theta(0.0, 5.0, pos[0], pos[1])
    bearing = get_bearing()
    
    left_motor.setVelocity(MAX_SPEED)
    right_motor.setVelocity(MAX_SPEED)
    
    error = angle_diff(angle_to_goal, bearing)
    if abs(error) > 5.0:
        turn_to(GOAL)
        
    if right_front < 990:
        if mode == "bug0":
            obs_circling_bug0()
        elif mode == "bug1":
            obs_circling_bug1()
        elif mode == "bug2":
            obs_circling_bug2()


def open_path_to_goal(pos, target):
    """Evaluates if the direct line-of-sight toward the target is clear of obstacles."""
    front = (ds[0].getValue() + ds[5].getValue()) / 2
    left = ds[1].getValue()
    right = ds[2].getValue()
    left_front = ds[4].getValue()
    
    if front > 910 and left > 940 and left_front > 990:
        return True
    return False


# --- ALGORITHMIC STATE BEHAVIORS ---

def obs_circling_bug0():
    """Implements Bug 0 obstacle-boundary circumnavigation."""
    while robot.step(TIME_STEP) != -1:
        front = (ds[0].getValue() + ds[5].getValue()) / 2
        right = ds[2].getValue()
        right_front = ds[3].getValue()
        left = ds[1].getValue()
        left_corner = ds[4].getValue()
        
        pos = gps.getValues()
        path_to_door.append([pos[0], pos[1]])  
        path_points.append([pos[0], pos[1]])
        bearing = get_bearing()
        
        # Obstacle avoidance maneuvering
        if front < 1000 or right_front < 990:
            if left_corner < 990:
                left_motor.setVelocity(0.8 * MAX_SPEED)
                right_motor.setVelocity(0.1 * MAX_SPEED)
            else:
                right_motor.setVelocity(0.7 * MAX_SPEED)
                left_motor.setVelocity(0.1 * MAX_SPEED)
        else:
            if (right < 985 and front > 990 and right_front > 990) or (left < 990 and front > 990):
                left_motor.setVelocity(1.0 * MAX_SPEED)
                right_motor.setVelocity(1.0 * MAX_SPEED)
            elif right > 990 and right_front > 990 and left_corner > 990:
                left_motor.setVelocity(0.6 * MAX_SPEED)
                right_motor.setVelocity(0.1 * MAX_SPEED)
                
            if right_front < 800:
                right_motor.setVelocity(0.6 * MAX_SPEED)
                left_motor.setVelocity(0.15 * MAX_SPEED)
                
        # Condition to exit obstacle-circling mode and resume straight path
        left_obs = (right > 990 and left < 1000 and left_corner > 990 and 170 < bearing < 200 and front > 990)
        right_obs = (right < 1000 and left > 990 and left_corner > 990 and 170 < bearing < 200 and front > 990)
        
        if right_obs or left_obs:
            break
            
    turn_to(GOAL)


def obs_circling_bug1(closest_goal=None):
    """Implements Bug 1 obstacle-boundary exploration and departure control."""
    global STATE 
    
    if closest_goal is None:
        closest_goal = [0.0, 0.0]
        
    starting_point = gps.getValues()
    closest_point = [100.0, 100.0]
    full_circle = False
    
    while robot.step(TIME_STEP) != -1:
        front = (ds[0].getValue() + ds[5].getValue()) / 2
        right = ds[2].getValue()
        right_front = ds[3].getValue()
        left_corner = ds[4].getValue()
        
        pos = gps.getValues()
        path_to_door.append([pos[0], pos[1]])  
        path_points.append([pos[0], pos[1]])
        
        if front < 750:
            left_motor.setVelocity(0.1 * MAX_SPEED)
            right_motor.setVelocity(0.8 * MAX_SPEED)
        elif left_corner < 990 and right > 990:
            left_motor.setVelocity(0.1 * MAX_SPEED)
            right_motor.setVelocity(0.8 * MAX_SPEED)
        else:
            if right < 900 and front > 800:
                left_motor.setVelocity(1.0 * MAX_SPEED)
                right_motor.setVelocity(1.0 * MAX_SPEED)
            elif right > 900 and front > 997:
                right_motor.setVelocity(0.1 * MAX_SPEED)
                left_motor.setVelocity(0.7 * MAX_SPEED)
            elif front < 1000 and right_front > 990:
                right_motor.setVelocity(0.7 * MAX_SPEED)
                left_motor.setVelocity(0.1 * MAX_SPEED)
                
        # Trajectory analysis during circumnavigation
        if STATE == 0 and distance(pos, GOAL) < distance(closest_point, GOAL):
            closest_point = pos
            
        if STATE == 1 and distance(pos, closest_goal) < 0.1:
            STATE = 0
            break    
            
        if STATE == 0 and distance(pos, starting_point) < 0.1 and full_circle:
            print("Completed full circle around obstacle.")
            STATE = 1
            print(f"[obs_circling] target depart coordinates: ({closest_point[0]:.2f}, {closest_point[1]:.2f})")
            obs_circling_bug1(closest_point)
            break
        elif distance(pos, starting_point) > 0.2:
            full_circle = True 
            
    turn_to(GOAL)


def obs_circling_bug2():
    """Implements Bug 2 trajectory intersection departure control (M-line checks)."""
    while robot.step(TIME_STEP) != -1:
        front = (ds[0].getValue() + ds[5].getValue()) / 2
        right = ds[2].getValue()
        right_front = ds[3].getValue()
        left = ds[1].getValue()
        left_corner = ds[4].getValue()
        
        pos = gps.getValues()
        path_to_door.append([pos[0], pos[1]])  
        path_points.append([pos[0], pos[1]])
        bearing = get_bearing()
        
        if front < 1000:
            if left_corner < 990:
                left_motor.setVelocity(0.8 * MAX_SPEED)
                right_motor.setVelocity(0.1 * MAX_SPEED)
            else:
                right_motor.setVelocity(0.7 * MAX_SPEED)
                left_motor.setVelocity(-0.1 * MAX_SPEED)
        else:
            if (right < 870 and front > 990 and right_front > 990) or (left < 990 and front > 990):
                left_motor.setVelocity(1.0 * MAX_SPEED)
                right_motor.setVelocity(1.0 * MAX_SPEED)
            elif right > 990 and right_front > 990:
                left_motor.setVelocity(0.7 * MAX_SPEED)
                right_motor.setVelocity(0.1 * MAX_SPEED)
                
            if right_front < 990:
                right_motor.setVelocity(0.7 * MAX_SPEED)
                left_motor.setVelocity(0.1 * MAX_SPEED)
                
        # Intersection condition with original line-of-sight path (M-line check)
        m_line_intercept = right < 1000 and left > 990 and left_corner > 990 and front > 990
        if is_on_M_line(pos[0], pos[1]) and m_line_intercept and bearing < 150:
            break
            
    turn_to(GOAL)


# --- MAIN PIPELINES ---

def wall_following():
    """Phase 2: Transitions robot to right-hand wall following until reaching target cabinet."""
    print("Transitioning to Phase 2: Wall-Following to Cabinet...")
    
    while robot.step(TIME_STEP) != -1:
        pos = gps.getValues()
        path_wall_following.append([pos[0], pos[1]]) 
        path_points.append([pos[0], pos[1]])
        
        if distance([pos[0], pos[1]], CABINET_GOAL) < 0.3:
            break
            
        front = (ds[0].getValue() + ds[5].getValue()) / 2
        right = ds[2].getValue()
        right_front = ds[3].getValue()
        left_front = ds[4].getValue()
        
        if front < 750:
            left_motor.setVelocity(0.1 * MAX_SPEED)
            right_motor.setVelocity(0.8 * MAX_SPEED)
        elif left_front < 990 and right > 990:
            left_motor.setVelocity(0.1 * MAX_SPEED)
            right_motor.setVelocity(0.8 * MAX_SPEED)
        else:
            if right < 900 and front > 800:
                left_motor.setVelocity(1.0 * MAX_SPEED)
                right_motor.setVelocity(1.0 * MAX_SPEED)
            elif right > 900:
                right_motor.setVelocity(0.1 * MAX_SPEED)
                left_motor.setVelocity(0.7 * MAX_SPEED)
                
    stop()
    print("Reached final goal (Cabinet)!")


def run_bug_navigator(execution_mode="bug0"):
    """Main execution loop driving the robot to the doorway using selected Bug mode."""
    global mode
    mode = execution_mode
    print(f"Starting Navigation Strategy: {mode}")
    
    turn_to(GOAL)
    go_straight()
    
    while robot.step(TIME_STEP) != -1:
        pos = gps.getValues()
        path_to_door.append([pos[0], pos[1]]) 
        path_points.append([pos[0], pos[1]])
        
        # Check if doorway goal region is reached
        if -0.5 < pos[0] < 0.5 and pos[1] > 4.62:
            print("Successfully reached transitional goal (doorway)!")
            break
            
        if open_path_to_goal(pos, GOAL):   
            go_straight()
        else:
            print("Obstacle encountered. Initiating boundary-following state...")
            if mode == "bug0":
                obs_circling_bug0()
            elif mode == "bug1":
                obs_circling_bug1()
            elif mode == "bug2":
                obs_circling_bug2()
                
            turn_to(GOAL)
            
    stop()
    wall_following()


def save_path(path, filename):
    """Saves structural path coordinates array to a CSV file."""
    with open(filename, mode='w', newline='') as f:
        writer = csv.writer(f)
        writer.writerow(["x", "y"])
        writer.writerows(path)


# === RUN CONTROLLER ===
if __name__ == "__main__":
    # Choose execution mode: "bug0", "bug1", or "bug2"
    RUN_MODE = "bug0" 
    
    run_bug_navigator(RUN_MODE)
    
    # Save files
    save_path(path_points, f"{RUN_MODE}_path.csv")
    save_path(path_to_door, f"{RUN_MODE}_door.csv")
    save_path(path_wall_following, f"{RUN_MODE}_cabinet.csv")
    
    # Analyze Path Metrics
    path_len = calculate_path_length(path_points)
    max_err = calculate_deviation(path_points, path_points[0], GOAL)
    print("\n--- PERFORMANCE SUMMARY ---")
    print(f"Strategy Model:   {RUN_MODE.upper()}")
    print(f"Path Length:      {path_len:.2f} meters")
    print(f"Maximum Error:    {max_err:.2f} meters")