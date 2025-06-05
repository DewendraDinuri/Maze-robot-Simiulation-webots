from controller import Robot, Motor, PositionSensor
import heapq

robot = Robot()
TIME_STEP = int(robot.getBasicTimeStep())

motorL = robot.getDevice("motorL")
motorR = robot.getDevice("motorR")
motorL.setPosition(float('inf'))
motorR.setPosition(float('inf'))
MAX_SPEED = 3.0

left_sensor = motorL.getPositionSensor()
right_sensor = motorR.getPositionSensor()
left_sensor.enable(TIME_STEP)
right_sensor.enable(TIME_STEP)

WHEEL_RADIUS = 0.0205
AXLE_LENGTH = 0.053
CELL_SIZE = 0.1
GRID_SIZE = 20

goal_pos = [18, 18]
direction = (0, 1)
dir_map = {(1,0):0, (0,1):1, (-1,0):2, (0,-1):3}
robot_pos = [10, 10]

grid = [[0 for _ in range(GRID_SIZE)] for _ in range(GRID_SIZE)]
for i in range(GRID_SIZE):
    grid[0][i] = grid[GRID_SIZE-1][i] = 1
    grid[i][0] = grid[i][GRID_SIZE-1] = 1

def dijkstra(grid, start, goal):
    distances = {tuple(start): 0}
    parent = {tuple(start): None}
    visited = set()
    pq = [(0, tuple(start))]
    while pq:
        dist, current = heapq.heappop(pq)
        if current in visited:
            continue
        visited.add(current)
        if current == tuple(goal):
            break
        r, c = current
        for dr, dc in [(-1,0),(1,0),(0,-1),(0,1)]:
            nr, nc = r+dr, c+dc
            if 0 <= nr < GRID_SIZE and 0 <= nc < GRID_SIZE and grid[nr][nc] == 0:
                new_dist = dist + 1
                if (nr, nc) not in distances or new_dist < distances[(nr, nc)]:
                    distances[(nr, nc)] = new_dist
                    parent[(nr, nc)] = current
                    heapq.heappush(pq, (new_dist, (nr, nc)))
    path = []
    current = tuple(goal)
    while current:
        path.append(current)
        current = parent.get(current)
    path.reverse()
    return path

def stop_motors():
    motorL.setVelocity(0)
    motorR.setVelocity(0)

def turn_left():
    for _ in range(15):
        motorL.setVelocity(-0.5 * MAX_SPEED)
        motorR.setVelocity(0.5 * MAX_SPEED)
        robot.step(TIME_STEP)

def turn_right():
    for _ in range(15):
        motorL.setVelocity(0.5 * MAX_SPEED)
        motorR.setVelocity(-0.5 * MAX_SPEED)
        robot.step(TIME_STEP)

def move_forward_distance(distance):
    motorL.setVelocity(MAX_SPEED)
    motorR.setVelocity(MAX_SPEED)
    start_left = left_sensor.getValue()
    start_right = right_sensor.getValue()
    while robot.step(TIME_STEP) != -1:
        dl = (left_sensor.getValue() - start_left) * WHEEL_RADIUS
        dr = (right_sensor.getValue() - start_right) * WHEEL_RADIUS
        if (dl + dr) / 2 >= distance:
            break
    stop_motors()

def update_position(move):
    robot_pos[0] += move[0]
    robot_pos[1] += move[1]

path = dijkstra(grid, robot_pos, goal_pos)
path_index = 0
print("Mapped path:", path)

while robot.step(TIME_STEP) != -1:
    if path_index >= len(path):
        stop_motors()
        break
    target = path[path_index]
    move = (target[0] - robot_pos[0], target[1] - robot_pos[1])
    if move == (0, 0):
        path_index += 1
        continue
    if move != direction:
        current_dir = dir_map[direction]
        target_dir = dir_map.get(move)
        turns = (target_dir - current_dir) % 4
        if turns == 1:
            turn_right()
        elif turns == 2:
            turn_right(); turn_right()
        elif turns == 3:
            turn_left()
        direction = move
    move_forward_distance(CELL_SIZE)
    update_position(move)
    path_index += 1
