# runThrough.py - Improved robot control with smarter exploration and red line logic

from controller import Robot, Camera, Motor, DistanceSensor
from controller2 import detect_red_line, a_star, PID
import math

robot = Robot()
timeStep = 64

# Devices
camera = robot.getDevice('cam')
camera.enable(timeStep)
camera_width = camera.getWidth()
camera_height = camera.getHeight()

sensor_names = ['senL1', 'senL2', 'senF', 'senR1', 'senR2']
sensors = [robot.getDevice(name) for name in sensor_names]
for sensor in sensors:
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
stuck_counter = 0
grid_step_counter = 0

# Movement logic
def center_and_avoid():
    left1 = sensors[0].getValue()
    left2 = sensors[1].getValue()
    front = sensors[2].getValue()
    right1 = sensors[3].getValue()
    right2 = sensors[4].getValue()

    forward_speed = 4.0
    turn_speed = 0.0

    obstacle = front > 950
    left_obstacle = left1 > 900 or left2 > 900
    right_obstacle = right1 > 900 or right2 > 900

    if obstacle:
        if not left_obstacle:
            turn_speed = 2.0
        elif not right_obstacle:
            turn_speed = -2.0
        else:
            turn_speed = 5.0
            forward_speed = -1.0
    else:
        if left1 > right2:
            turn_speed = 0.5
        elif right2 > left1:
            turn_speed = -0.5

    motorL.setVelocity(clamp_velocity(forward_speed - turn_speed))
    motorR.setVelocity(clamp_velocity(forward_speed + turn_speed))

# Clamp function
def clamp_velocity(v, limit=6.0):
    return max(-limit, min(limit, v))

frame_count = 0

while robot.step(timeStep) != -1:
    frame_count += 1
    image = camera.getImage() if frame_count % 5 == 0 else None
    red_detected = detect_red_line(image, camera_width, camera_height) if image else False

    print("Red line detected:", red_detected)
    print("Started:", started, "Exploring:", exploring, "Racing:", racing_mode)

    if red_detected and not started:
        started = True
        exploring = True
        print("Start detected. Begin exploration.")
        continue

    if red_detected and exploring:
        if len(position_log) < 20 or current_pos == position_log[0]:
            print("Red detected too early, continue exploring...")
            continue
        print("End of first loop. Switching to racing mode.")
        exploring = False
        racing_mode = True
        optimized_path = a_star(position_log[0], position_log[-1], graph)
        path_index = 0
        current_pos = position_log[0]
        continue

    if exploring:
        print("Exploring... Moving forward")
        obstacle = sensors[2].getValue() > 950
        left_blocked = sensors[0].getValue() > 900 or sensors[1].getValue() > 900
        right_blocked = sensors[3].getValue() > 900 or sensors[4].getValue() > 900

        if not obstacle:
            stuck_counter = 0
            pos = current_pos
            if pos not in position_log:
                position_log.append(pos)
                neighbors = []
                forward = (pos[0] + 1, pos[1])
                if forward not in graph.get(pos, []):
                    neighbors.append(forward)
                if not left_blocked:
                    neighbors.append((pos[0], pos[1] + 1))
                if not right_blocked:
                    neighbors.append((pos[0], pos[1] - 1))
                graph.setdefault(pos, []).extend(neighbors)
            current_pos = (current_pos[0] + 1, current_pos[1])
            center_and_avoid()
        else:
            stuck_counter += 1
            if stuck_counter > 10:
                print("Path blocked. Recomputing using A*")
                for node in reversed(position_log):
                    if node != current_pos and node not in graph.get(current_pos, []):
                        temp_path = a_star(current_pos, node, graph)
                        if temp_path:
                            optimized_path = temp_path
                            path_index = 0
                            racing_mode = True
                            exploring = False
                            break
                stuck_counter = 0

    elif racing_mode and path_index < len(optimized_path):
        front_obstacle = sensors[2].getValue() > 950
        if front_obstacle:
            print("Obstacle detected in racing mode! Trying to escape...")
            motorL.setVelocity(2.0)
            motorR.setVelocity(2.0)
            robot.step(timeStep * 5)

            print("Recomputing path using A*...")
            for node in reversed(position_log):
                if node != current_pos:
                    temp_path = a_star(current_pos, node, graph)
                    if temp_path:
                        optimized_path = temp_path
                        path_index = 0
                        print("New path found!")
                        break
            else:
                print("No new path found, switching back to exploration...")
                racing_mode = False
                exploring = True
            continue

        target = optimized_path[path_index]
        dx = target[0] - current_pos[0]
        dy = target[1] - current_pos[1]
        target_angle = math.atan2(dy, dx)

        if path_index > 0:
            prev = optimized_path[path_index - 1]
            heading = math.atan2(current_pos[1] - prev[1], current_pos[0] - prev[0])
        else:
            heading = 0.0

        heading_error = target_angle - heading
        heading_error = math.atan2(math.sin(heading_error), math.cos(heading_error))
        correction = pid.compute(heading_error)
        correction = max(min(correction, 2.0), -2.0)

        motorL.setVelocity(clamp_velocity(6.0 - correction))
        motorR.setVelocity(clamp_velocity(6.0 + correction))

        print(f"[Racing] Target: {target}, Pos: {current_pos}, Correction: {correction}")

        grid_step_counter += 1
        if grid_step_counter > 30:
            current_pos = target
            path_index += 1
            print("Simulated step reached. Moving to next waypoint.")
            grid_step_counter = 0

    elif racing_mode and path_index >= len(optimized_path):
        print("Temporary path finished. Resuming exploration.")
        racing_mode = False
        exploring = True

    else:
        motorL.setVelocity(0)
        motorR.setVelocity(0)
