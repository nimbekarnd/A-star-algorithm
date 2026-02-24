# A* Algorithm

Implementation of the A* path planning algorithm for a rigid robot navigating a 2D obstacle map.

---

## Table of Contents

- [Overview](#overview)
- [Python Implementation](#python-implementation)
- [C++ Implementation](#c-implementation)

---

## Overview

The algorithm finds the optimal path for a robot from a start position to a goal position on a 300×200 map containing the following obstacles: circle, ellipse, two triangles, a rhombus, a square, and a rod. The robot has a configurable radius and obstacle clearance.

---

## Python Implementation

### Requirements

Install dependencies using a virtual environment (recommended):

```bash
python3 -m venv .venv
source .venv/bin/activate
pip install opencv-python numpy
```

Or using the system package manager:

```bash
sudo apt install python3-opencv python3-numpy
```

### Running

```bash
cd Python
python3 main.py
```

You will be prompted to enter the following parameters:

```
--- Robot Parameters ---
  Robot radius       (int, >= 0) :
  Obstacle clearance (int, >= 0) :

--- Start Position ---
  Start row   (float, <min> – <max>) :
  Start col   (float, <min> – <max>) :
  Start angle (int, 0/30/60/.../330) :

--- Goal Position ---
  Goal row    (float, <min> – <max>) :
  Goal col    (float, <min> – <max>) :
```

> **Note:** The valid position range is shown dynamically based on the radius and clearance entered. Start and goal positions must lie within this range and outside any obstacle.

### Parameter Reference

| Parameter | Type | Constraints |
|-----------|------|-------------|
| Robot radius | int | >= 0 |
| Clearance | int | >= 0 |
| Start / Goal row | float | `1 + radius + clearance` to `200 - radius - clearance` |
| Start / Goal col | float | `1 + radius + clearance` to `300 - radius - clearance` |
| Start angle | int | Multiple of 30, range 0–330 |

### Output

Results are saved to the `output/` folder:

| File | Description |
|------|-------------|
| `astar_algoritm_in_python.avi` | Animated video of exploration and path |
| `astar_python_path.png` | Final frame showing the complete path |

### Example

Parameters used:

![Parameters](output/Parameter_for_python_test.png)

Result:

![A* Path](output/astar_python_path.png)

---

## C++ Implementation

> In Progress

---
