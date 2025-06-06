"""robot_controller controller."""

# You may need to import more classes/modules from the controller
# depending on your robot's functionality
from controller import Robot, Camera, Motor

# Custom function to Map sensor input values
def map_value(value, in_min, in_max, out_min, out_max):
    """
    Maps a value from one range to another range.

    Args:
        value (float/int): The value to map.
        in_min (float/int): The minimum value of the input range.
        in_max (float/int): The maximum value of the input range.
        out_min (float/int): The minimum value of the output range.
        out_max (float/int): The maximum value of the output range.

    Returns:
        float: The mapped value.
    """
    # Calculate the ratio of the value's position in the input range
    in_range = in_max - in_min
    if in_range == 0: # Avoid division by zero if input range is just a single point
        return out_min # Or raise an error, depending on desired behavior

    ratio = (value - in_min) / in_range

    # Map the ratio to the output range
    out_range = out_max - out_min
    mapped_value = ratio * out_range + out_min

    return mapped_value

# Create the Robot instance.
robot = Robot()

# Get the time step of the current world.
TIME_STEP = int(robot.getBasicTimeStep())

# Get Distance Sensors
sensors = ['senF', 'senR1', 'senR2', 'senL1', 'senL2']
sensor_list = []

for name in sensors:
    sensor = robot.getDevice(name)
    if sensor:
        sensor_list.append(sensor)
        # --- 2. Enable the Distance Sensor ---
        # Sampling period should typically be TIME_STEP or a multiple.
        sensor.enable(TIME_STEP)
        print(f"Enabled sensor: {name}, minRange: {sensor.getMinValue():.2f}m, maxRange: {sensor.getMaxValue():.2f}m")
    else:
        print(f"Error: Distance sensor '{name}' not found.")
        
front_thres = 0.2
side1_thres = 0.2 
side2_thres = 0.2
# Get the camera and set variables
camera_name = "cam"
camera = robot.getDevice(camera_name)
camera.enable(TIME_STEP)

cam_width = camera.getWidth()
cam_height = camera.getHeight()

# Get the motor devices
motorL = robot.getDevice("motorL")
motorR = robot.getDevice("motorR")

# Check if motors are found
if motorL is None:
    print("Motor 'motorL' not found.")
if motorR is None:
    print("Motor 'motorR' not found.")

if motorL and motorR:
    # Set the motors to velocity control mode
    # This is done by setting the position to infinity
    motorL.setPosition(float('inf'))
    motorR.setPosition(float('inf'))

    # Set an initial velocity (e.g., 50% of max speed)
    # The velocity unit is rad/s
    # You might need to adjust this value depending on your robot's max motor speed
    
#    left_speed = 0.0  # radians per second
#    right_speed = 0.0 # radians per second

#    motorL.setVelocity(left_speed)
#    motorR.setVelocity(right_speed)
    
    print("Robot initialized.")

# Main loop:
# - perform simulation steps until Webots is stopping the controller
while robot.step(TIME_STEP) != -1:
    sensor_readings = {}
    for sensor in sensor_list:
        # getValue() returns the raw sensor reading (e.g., 0-1024)
        raw_value = sensor.getValue()
        # getDistance() converts the raw value to meters using the lookupTable
        distance_in_meters = map_value(sensor.getValue(), 0, 1023, 0.1, 0.3)
        sensor_readings[sensor.getName()] = distance_in_meters
        print(f"Sensor '{sensor.getName()}': {distance_in_meters:.3f} m (raw: {raw_value:.0f})")
        
#    forward_speed = 0.5 # Default speed
#    turn_speed = 0.0    # Default turn
    
    left_dist = sensor_readings.get("senL2", float('inf'))
    front_left_dist = sensor_readings.get("senL1", float('inf'))
    front_dist = sensor_readings.get("senF", float('inf'))
    front_right_dist = sensor_readings.get("senR1", float('inf'))
    right_dist = sensor_readings.get("senR2", float('inf'))
    
    # Check if a front obstacle is detected
    if front_dist > front_thres:
        print("Front obstacle! Turning right.")
        turn_speed = 2 # Turn left (positive angular velocity)
        forward_speed = 3 # Slow down
    elif front_left_dist > side1_thres:
        print("Front-left obstacle! Turning right.")
        turn_speed = -1 # Turn right
        forward_speed = 5 # Slow down
    elif front_right_dist > side1_thres:
        print("Front-right obstacle! Turning left.")
        turn_speed = 1 # Turn left (positive angular velocity)
        forward_speed = 5 # Slow down
    elif left_dist > side2_thres:
        print("Left obstacle! Turning right.")
        turn_speed = -1 # Turn right
        forward_speed = 7 # Slow down
    elif right_dist > side2_thres:
        print("Right obstacle! Turning left.")
        turn_speed = 1 # Turn left (positive angular velocity)
        forward_speed = 7 # Slow down
    else:
        # If no immediate obstacle, move forward
        print("Path clear. Moving forward.")
        forward_speed = 10
        turn_speed = 0.0    

    motorL.setVelocity(forward_speed - turn_speed)
    motorR.setVelocity(forward_speed + turn_speed)