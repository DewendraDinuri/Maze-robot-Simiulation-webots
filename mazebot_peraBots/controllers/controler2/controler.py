# controler2.py
from controller import Robot

robot = Robot()
TIME_STEP = int(robot.getBasicTimeStep())

# Devices
motorL = robot.getDevice("motorL")
motorR = robot.getDevice("motorR")
motorL.setPosition(float('inf'))
motorR.setPosition(float('inf'))
motorL.setVelocity(0.0)
motorR.setVelocity(0.0)

# Example: Predefined path actions (can be replaced with Dijkstra-based dynamic movement)
def move_forward(duration=10, speed=6.0):
    motorL.setVelocity(speed)
    motorR.setVelocity(speed)
    for _ in range(duration):
        robot.step(TIME_STEP)

def turn_left(duration=5, speed=4.0):
    motorL.setVelocity(-speed)
    motorR.setVelocity(speed)
    for _ in range(duration):
        robot.step(TIME_STEP)

def turn_right(duration=5, speed=4.0):
    motorL.setVelocity(speed)
    motorR.setVelocity(-speed)
    for _ in range(duration):
        robot.step(TIME_STEP)

# Path-following logic example (mocked)
move_forward(20)
turn_left(10)
move_forward(20)
turn_right(10)
move_forward(20)

motorL.setVelocity(0)
motorR.setVelocity(0)
print("Path following complete.")
