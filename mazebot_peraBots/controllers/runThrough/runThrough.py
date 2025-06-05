"""Improved robot_controller.py for obstacle avoidance with distance sensors."""

from controller import Robot

def map_value(value, in_min, in_max, out_min, out_max):
    """Maps a value from one range to another range."""
    in_range = in_max - in_min
    if in_range == 0:
        return out_min
    ratio = (value - in_min) / in_range
    out_range = out_max - out_min
    return ratio * out_range + out_min

# Initialize robot and devices
robot = Robot()
TIME_STEP = int(robot.getBasicTimeStep())

# Sensor names
sensor_names = ['senF', 'senR1', 'senR2', 'senL1', 'senL2']
sensors = {}

for name in sensor_names:
    sensor = robot.getDevice(name)
    if sensor:
        sensor.enable(TIME_STEP)
        sensors[name] = sensor
        print(f"Enabled sensor: {name}, minRange: {sensor.getMinValue():.2f}m, maxRange: {sensor.getMaxValue():.2f}m")
    else:
        print(f"Warning: Sensor '{name}' not found!")

# Motors
motorL = robot.getDevice("motorL")
motorR = robot.getDevice("motorR")

if motorL is None or motorR is None:
    print("Error: One or both motors not found!")
else:
    motorL.setPosition(float('inf'))
    motorR.setPosition(float('inf'))
    motorL.setVelocity(0.0)
    motorR.setVelocity(0.0)
    print("Motors initialized.")

# Threshold distances (meters)
FRONT_THRESHOLD = 0.2
SIDE_THRESHOLD_1 = 0.2
SIDE_THRESHOLD_2 = 0.2

# Main control loop
while robot.step(TIME_STEP) != -1:
    sensor_distances = {}
    for name, sensor in sensors.items():
        raw_val = sensor.getValue()
        # Map raw sensor value (0-1023) to distance approx 0.1-0.3m
        dist = map_value(raw_val, 0, 1023, 0.1, 0.3)
        sensor_distances[name] = dist
        print(f"Sensor '{name}': {dist:.3f} m (raw: {raw_val:.0f})")

    # Read distances with defaults
    left_dist = sensor_distances.get("senL2", float('inf'))
    front_left_dist = sensor_distances.get("senL1", float('inf'))
    front_dist = sensor_distances.get("senF", float('inf'))
    front_right_dist = sensor_distances.get("senR1", float('inf'))
    right_dist = sensor_distances.get("senR2", float('inf'))

    # Initialize speeds
    forward_speed = 0.0
    turn_speed = 0.0

    # Obstacle avoidance logic (adjust thresholds and speeds as needed)
    if front_dist < FRONT_THRESHOLD:
        print("Front obstacle detected! Turning right.")
        forward_speed = 3.0
        turn_speed = 2.0  # turn right
    elif front_left_dist < SIDE_THRESHOLD_1:
        print("Front-left obstacle detected! Turning right.")
        forward_speed = 5.0
        turn_speed = 1.0  # turn right
    elif front_right_dist < SIDE_THRESHOLD_1:
        print("Front-right obstacle detected! Turning left.")
        forward_speed = 5.0
        turn_speed = -1.0  # turn left
    elif left_dist < SIDE_THRESHOLD_2:
        print("Left obstacle detected! Turning right.")
        forward_speed = 7.0
        turn_speed = 1.5  # turn right
    elif right_dist < SIDE_THRESHOLD_2:
        print("Right obstacle detected! Turning left.")
        forward_speed = 7.0
        turn_speed = -1.5  # turn left
    else:
        print("Path clear. Moving forward.")
        forward_speed = 10.0
        turn_speed = 0.0

    # Calculate motor velocities
    left_velocity = forward_speed - turn_speed
    right_velocity = forward_speed + turn_speed

    # Clamp motor velocities to safe limits (adjust max speed if needed)
    MAX_SPEED = 10.0
    left_velocity = max(min(left_velocity, MAX_SPEED), -MAX_SPEED)
    right_velocity = max(min(right_velocity, MAX_SPEED), -MAX_SPEED)

    # Set motor velocities
    motorL.setVelocity(left_velocity)
    motorR.setVelocity(right_velocity)
