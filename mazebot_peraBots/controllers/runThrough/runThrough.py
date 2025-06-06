# runThrough.py - Stabilized racing mode with position fix

from controller import Robot, Camera, Motor, DistanceSensor
from controller2 import detect_red_line, a_star, PID

robot = Robot()
timeStep = 64

# Devices
camera = robot.getDevice('cam')
if camera is None:
    print("Camera 'cam' not found!")
    exit(1)
camera.enable(timeStep)
camera_width = camera.getWidth()
camera_height = camera.getHeight()

sensor_names = ['senL1', 'senL2', 'senF', 'senR1', 'senR2']
sensors = [robot.getDevice(name) for name in sensor_names]
for i, sensor in enumerate(sensors):
    if sensor is None:
        print(f"Sensor '{sensor_names[i]}' not found!")
        exit(1)
    sensor.enable(timeStep)

motorL = robot.getDevice("motorL")
motorR = robot.getDevice("motorR")
motorL.setPosition(float('inf'))
motorR.setPosition(float('inf'))
motorL.setVelocity(0.0)
motorR.setVelocity(0.0)

# States
started = False
exploring = False
racing_mode = False
position_log = []
graph = {}
path_index = 0
optimized_path = []
current_pos = (0, 0)
pid = PID(1.0, 0.0, 0.1)

# Movement logic
def center_and_avoid():
    left = sensors[0].getValue()
    center = sensors[2].getValue()
    right = sensors[4].getValue()

    forward_speed = 4.0
    turn_speed = 0.0

    if center < 900:
        if left < right:
            turn_speed = 1.0
        else:
            turn_speed = -1.0

    motorL.setVelocity(forward_speed - turn_speed)
    motorR.setVelocity(forward_speed + turn_speed)

# Clamp function
def clamp_velocity(v, limit=6.0):
    return max(-limit, min(limit, v))

frame_count = 0

# Main loop
while robot.step(timeStep) != -1:
    frame_count += 1

    if frame_count % 5 == 0:
        image = camera.getImage()
        red_detected = detect_red_line(image, camera_width, camera_height)
    else:
        red_detected = False

    print("Red line detected:", red_detected)
    print("Started:", started, "Exploring:", exploring, "Racing:", racing_mode)

    if red_detected and not started:
        started = True
        exploring = True
        print("Start detected. Begin exploration.")

    elif red_detected and exploring:
        if len(position_log) < 5:
            print("Red detected too early, continue exploring...")
            continue
        print("End of first loop. Switching to racing mode.")
        exploring = False
        racing_mode = True
        optimized_path = a_star(position_log[0], position_log[-1], graph)
        path_index = 0
        current_pos = position_log[0]  # reset to starting position
        continue

    if exploring:
        print("Exploring... Moving forward")
        if sensors[2].getValue() > 950:
            pos = current_pos
            position_log.append(pos)
            graph.setdefault(pos, []).append((pos[0] + 1, pos[1]))
            current_pos = (current_pos[0] + 1, current_pos[1])
        center_and_avoid()

    elif racing_mode and path_index < len(optimized_path):
        target = optimized_path[path_index]
        error = target[0] - current_pos[0]
        correction = pid.compute(error)

        # Clamp correction
        correction = max(min(correction, 2.0), -2.0)

        left_speed = clamp_velocity(6.0 - correction)
        right_speed = clamp_velocity(6.0 + correction)

        motorL.setVelocity(left_speed)
        motorR.setVelocity(right_speed)

        print(f"[Racing] Target: {target}, Pos: {current_pos}, Correction: {correction}")

        # Update only if robot is close enough to target
        if abs(current_pos[0] - target[0]) < 1:
            current_pos = target
            path_index += 1
            print("Reached target. Moving to next waypoint.")
    else:
        motorL.setVelocity(0)
        motorR.setVelocity(0)