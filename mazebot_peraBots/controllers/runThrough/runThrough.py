
from controller import Robot, Motor, DistanceSensor
import heapq
import math

robot = Robot()
TIME_STEP = int(robot.getBasicTimeStep())

motorL = robot.getDevice("motorL")
motorR = robot.getDevice("motorR")
motorL.setPosition(float('inf'))
motorR.setPosition(float('inf'))
MAX_SPEED = 6.28

sensor_names = ['senF', 'senL1', 'senL2', 'senR1', 'senR2']
sensors = {}
for name in sensor_names:
    sensor = robot.getDevice(name)
    sensor.enable(TIME_STEP)
    sensors[name] = sensor

GRID_SIZE = 20
grid = [[0 for _ in range(GRID_SIZE)] for _ in range(GRID_SIZE)]
robot_pos = [10, 10]
goal_pos = [18, 18]

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

def move_forward(steps=8):
    for _ in range(steps):
        motorL.setVelocity(MAX_SPEED)
        motorR.setVelocity(MAX_SPEED)
        robot.step(TIME_STEP)

def turn_left(steps=8):
    for _ in range(steps):
        motorL.setVelocity(-0.5 * MAX_SPEED)
        motorR.setVelocity(0.5 * MAX_SPEED)
        robot.step(TIME_STEP)

def turn_right(steps=8):
    for _ in range(steps):
        motorL.setVelocity(0.5 * MAX_SPEED)
        motorR.setVelocity(-0.5 * MAX_SPEED)
        robot.step(TIME_STEP)

direction = (0, 1)
dir_map = {(1,0):0, (0,1):1, (-1,0):2, (0,-1):3}

mapped = False
while robot.step(TIME_STEP) != -1:
    if not mapped:
        for i in range(GRID_SIZE):
            grid[0][i] = 1
            grid[GRID_SIZE-1][i] = 1
            grid[i][0] = 1
            grid[i][GRID_SIZE-1] = 1
        mapped = True
        path = dijkstra(grid, robot_pos, goal_pos)
        path_index = 0
        print("Mapped path:", path)

    if path_index >= len(path):
        motorL.setVelocity(0)
        motorR.setVelocity(0)
        break

    target = path[path_index]
    move = (target[0] - robot_pos[0], target[1] - robot_pos[1])
    if move == (0, 0):
        path_index += 1
        continue

    if move != direction:
        current_dir = dir_map[direction]
        target_dir = dir_map[move]
        turns = (target_dir - current_dir) % 4
        if turns == 1:
            turn_right(10)
        elif turns == 2:
            turn_right(10)
            turn_right(10)
        elif turns == 3:
            turn_left(10)
        direction = move

    move_forward(10)
    robot_pos = list(target)
    path_index += 1
