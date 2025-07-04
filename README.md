# Maze-robot-Simiulation-webots

# 🤖 Webots Dijkstra Maze Robot

This project implements a fast, autonomous maze-solving robot using **Dijkstra's algorithm** inside the Webots simulation environment. The robot calculates the shortest path through a maze from start to goal and follows it using precise motor control. Designed for robotics competitions, educational projects, and AI path planning demonstrations.

---

## 🗂️ Project Structure

```
webots-dijkstra-maze-robot/
├── controllers/
│   ├── controller2/         # Robot movement controller
│   │   └── controller2.py   # Drives robot using Dijkstra path
│   └── runThrough/          # Path planning module
│       └── runthrough.py    # Dijkstra’s algorithm implementation
├── worlds/
│   └── undergraduate_exampleArena_new.wbt  # Maze simulation world
├── path.txt                 # Output path generated by Dijkstra algorithm
├── README.md                # This documentation file
```

---

## 🤖 Robot Description

- **Simulator**: [Webots](https://cyberbotics.com/)
- **Robot Name**: `robot`
- **Drive Type**: Differential drive (2-wheel)
- **Sensors**: 5 ultrasonic/distance sensors
  - `senF`, `senL1`, `senL2`, `senR1`, `senR2`
- **Motors**: 2 motors attached via `HingeJoint`
- **Controller Language**: Python

> ⚠️ If motors are not found, use `getDeviceByIndex()` to print available device names in `controller2.py`.

---

## ⚙️ How It Works

### 1. Dijkstra Path Calculation (`runthrough.py`)
- Uses a 2D grid to represent the maze: `0 = free space`, `1 = wall`
- Computes the shortest path from `start` to `goal`
- Writes the path to `path.txt` for the robot to follow

### 2. Robot Movement (`controller2.py`)
- Loads the path from `path.txt`
- Drives the robot through the maze based on grid coordinates
- Uses basic velocity control to move forward step-by-step

---

## 🚀 Getting Started

### Requirements

- Webots installed
- Python 3.x installed (e.g., via Anaconda)

### Run the Simulation

1. Open Webots and launch the world:
   ```
   worlds/undergraduate_exampleArena_new.wbt
   ```

2. Generate the shortest path using Dijkstra:
   ```bash
   cd controllers/runThrough
   python runthrough.py
   ```

3. In Webots, start the simulation.
   - The robot will use `controller2.py` to move using the generated path.

---

## 🧠 Customization

### 🔧 Maze Grid (inside `runthrough.py`)
You can modify your maze like this:

```python
grid = [
    [0, 1, 0, 0],
    [0, 1, 0, 1],
    [0, 0, 0, 0]
]
start = (0, 0)
goal = (2, 3)
```

### 🔍 Find Actual Motor Names

In `controller2.py`, to debug motor names:

```python
for i in range(robot.getNumberOfDevices()):
    print(robot.getDeviceByIndex(i).getName())
```

Then set:
```python
left_motor = robot.getDevice("actual_left_motor_name")
right_motor = robot.getDevice("actual_right_motor_name")
```

---

## 📸 Preview

> Maze and robot in Webots (add screenshot later)

```
[Insert maze_view.png here]
```

---

## 👩‍💻 Author

**Dinuri Dewendra**  
Undergraduate – Software Engineering  
University of Westminster (IIT Affiliated)  
GitHub: [github.com/your-username](https://github.com/your-username)

---

## 📜 License

This project is licensed under the MIT License.
