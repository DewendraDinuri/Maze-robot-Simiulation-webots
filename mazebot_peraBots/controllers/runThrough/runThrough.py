# runthrough.py
from controller import Robot
import heapq

# --- Dijkstra Pathfinding Function ---
def dijkstra(graph, start, goal):
    queue = [(0, start)]
    distances = {node: float('inf') for node in graph}
    distances[start] = 0
    previous = {node: None for node in graph}

    while queue:
        curr_distance, curr_node = heapq.heappop(queue)
        if curr_node == goal:
            break

        for neighbor, weight in graph[curr_node].items():
            distance = curr_distance + weight
            if distance < distances[neighbor]:
                distances[neighbor] = distance
                previous[neighbor] = curr_node
                heapq.heappush(queue, (distance, neighbor))

    path = []
    node = goal
    while node:
        path.append(node)
        node = previous[node]
    return path[::-1]

# --- Initialize Robot and Motors ---
robot = Robot()
TIME_STEP = int(robot.getBasicTimeStep())

motorL = robot.getDevice("motorL")
motorR = robot.getDevice("motorR")
motorL.setPosition(float('inf'))
motorR.setPosition(float('inf'))
motorL.setVelocity(0.0)
motorR.setVelocity(0.0)

# --- Define Maze as Graph (example 3x3 grid) ---
graph = {
    (0,0): {(0,1): 1, (1,0): 1},
    (0,1): {(0,0): 1, (0,2): 1},
    (0,2): {(0,1): 1, (1,2): 1},
    (1,0): {(0,0): 1, (2,0): 1},
    (1,2): {(0,2): 1, (2,2): 1},
    (2,0): {(1,0): 1, (2,1): 1},
    (2,1): {(2,0): 1, (2,2): 1},
    (2,2): {(2,1): 1, (1,2): 1},
}

start = (0,0)
goal = (2,2)
path = dijkstra(graph, start, goal)

# --- Movement Helper ---
def move_forward(speed=5.0):
    motorL.setVelocity(speed)
    motorR.setVelocity(speed)

def stop():
    motorL.setVelocity(0.0)
    motorR.setVelocity(0.0)

# --- Simulate Following the Path ---
for node in path:
    print(f"Moving to node {node}...")
    move_forward()
    for _ in range(20):  # simulate steps
        robot.step(TIME_STEP)
    stop()
    for _ in range(5):
        robot.step(TIME_STEP)

print("Reached the goal!")
