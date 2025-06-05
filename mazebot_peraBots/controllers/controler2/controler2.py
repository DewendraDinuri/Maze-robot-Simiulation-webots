
from controller import Robot, Motor
import heapq

# -------------------- Maze and Dijkstra -------------------- #
# Sample 5x5 grid: 0 = free path, 1 = wall
maze = [
    [0, 1, 0, 0, 0],
    [0, 1, 0, 1, 0],
    [0, 0, 0, 1, 0],
    [1, 1, 0, 1, 0],
    [0, 0, 0, 0, 0]
]

start = (0, 0)
goal = (4, 4)
rows, cols = len(maze), len(maze[0])

def dijkstra(maze, start, goal):
    distances = {start: 0}
    visited = set()
    pq = [(0, start)]
    parent = {start: None}

    while pq:
        dist, current = heapq.heappop(pq)
        if current in visited:
            continue
        visited.add(current)

        if current == goal:
            break

        r, c = current
        for dr, dc in [(-1,0),(1,0),(0,-1),(0,1)]:
            nr, nc = r+dr, c+dc
            neighbor = (nr, nc)
            if 0 <= nr < rows and 0 <= nc < cols and maze[nr][nc] == 0:
                new_dist = dist + 1
                if new_dist < distances.get(neighbor, float('inf')):
                    distances[neighbor] = new_dist
                    parent[neighbor] = current
                    heapq.heappush(pq, (new_dist, neighbor))

    path = []
    current = goal
    while current:
        path.append(current)
        current = parent.get(current)
    path.reverse()
    return path

# -------------------- Robot Setup -------------------- #
robot = Robot()
TIME_STEP = int(robot.getBasicTimeStep())

motorL = robot.getDevice("motorL")
motorR = robot.getDevice("motorR")

motorL.setPosition(float('inf'))
motorR.setPosition(float('inf'))

# Max speed depends on robot, here assumed 6.28 rad/s
MAX_SPEED = 6.28

def move_forward(duration_steps):
    for _ in range(duration_steps):
        motorL.setVelocity(MAX_SPEED)
        motorR.setVelocity(MAX_SPEED)
        robot.step(TIME_STEP)

def turn_left(duration_steps):
    for _ in range(duration_steps):
        motorL.setVelocity(-0.5 * MAX_SPEED)
        motorR.setVelocity(0.5 * MAX_SPEED)
        robot.step(TIME_STEP)

def turn_right(duration_steps):
    for _ in range(duration_steps):
        motorL.setVelocity(0.5 * MAX_SPEED)
        motorR.setVelocity(-0.5 * MAX_SPEED)
        robot.step(TIME_STEP)

# -------------------- Path Execution -------------------- #
path = dijkstra(maze, start, goal)
print("Planned Path:", path)

# Movement logic: assuming each cell = 1 move_forward, turns decided by direction
direction = (1, 0)  # Facing down initially
dir_map = {(1,0):0, (0,1):1, (-1,0):2, (0,-1):3}  # down, right, up, left
for i in range(1, len(path)):
    curr = path[i-1]
    next = path[i]
    move = (next[0]-curr[0], next[1]-curr[1])
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

    move_forward(20)
    direction = move

# Stop motors after reaching goal
motorL.setVelocity(0)
motorR.setVelocity(0)
