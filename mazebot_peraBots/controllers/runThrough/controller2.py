# controller2.py - Updated utility module

from controller import Camera
import heapq

# PID setup
class PID:
    def __init__(self, kp, ki, kd):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.prev_error = 0
        self.integral = 0

    def compute(self, error):
        self.integral += error
        derivative = error - self.prev_error
        self.prev_error = error
        return self.kp * error + self.ki * self.integral + self.kd * derivative

# A* utilities
def heuristic(a, b):
    return abs(a[0] - b[0]) + abs(a[1] - b[1])

def a_star(start, goal, graph):
    open_set = [(0, start)]
    came_from = {}
    g_score = {start: 0}
    f_score = {start: heuristic(start, goal)}

    while open_set:
        _, current = heapq.heappop(open_set)
        if current == goal:
            path = []
            while current in came_from:
                path.append(current)
                current = came_from[current]
            path.reverse()
            return path
        for neighbor in graph.get(current, []):
            tentative_g = g_score[current] + 1
            if neighbor not in g_score or tentative_g < g_score[neighbor]:
                came_from[neighbor] = current
                g_score[neighbor] = tentative_g
                f_score[neighbor] = tentative_g + heuristic(neighbor, goal)
                heapq.heappush(open_set, (f_score[neighbor], neighbor))
    return []

# Red line detection
def detect_red_line(image_data, width, height):
    red_pixel_count = 0
    threshold = 20  # More sensitive
    for y in range(height):
        for x in range(width):
            r = Camera.imageGetRed(image_data, width, x, y)
            g = Camera.imageGetGreen(image_data, width, x, y)
            b = Camera.imageGetBlue(image_data, width, x, y)
            if r > 100 and g < 70 and b < 70:
                red_pixel_count += 1
    return red_pixel_count > threshold
