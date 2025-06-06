# runThrough.py - Final updated version for red line, movement, and racing mode

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
    left2 = sensors[1].getValue()
    center = sensors[2].getValue()
    right1 = sensors[3].getValue()
    right2 = sensors[4].getValue()

    forward_speed = 4.0
    turn_speed = 0.0

    if center < 900:
        if left + left2 < right1 + right2:
            turn_speed = 1.5  # sharper right
        else:
            turn_speed = -1.5  # sharper left

    motorL.setVelocity(forward_speed - turn_speed)
    motorR.setVelocity(forward_speed + turn_speed)

# Main loop
while robot.step(timeStep) != -1:
    image = camera.getImage()
    red_detected = detect_red_line(image, camera_width, camera_height)

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
        continue

    if exploring:
        print("Exploring... Moving forward")
        pos = current_pos
        position_log.append(pos)
        graph.setdefault(pos, []).append((pos[0] + 1, pos[1]))  # dummy neighbor
        current_pos = (current_pos[0] + 1, current_pos[1])
        center_and_avoid()

    elif racing_mode and path_index < len(optimized_path):
        target = optimized_path[path_index]
        error = target[0] - current_pos[0]
        correction = pid.compute(error)

        motorL.setVelocity(6.0 - correction)
        motorR.setVelocity(6.0 + correction)

        current_pos = (current_pos[0] + 1, current_pos[1])
        if current_pos == target:
            path_index += 1
    else:
        motorL.setVelocity(0)
        motorR.setVelocity(0)