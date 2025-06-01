**Trace 2D shapes in 3D space using a simulated UFactory xArm 7 (7-DoF) robot with MoveIt 2 and RViz in ROS 2 (Humble).**

This package loads a list of 2D shapes from a JSON file, each defined with 2D vertices and a 3D starting pose. The robot traces each shape on its respective 3D plane using MoveIt Cartesian path planning.

---

## Features

- Load arbitrary 2D shapes from a JSON configuration
- Specify a 3D starting pose (position + orientation) for each shape
- Compute Cartesian trajectories using MoveIt 2
- Simulate motion in RViz with a virtual xArm7 robot
- Easily extensible for real robot or Gazebo integration

---

## Project Structure

```
shape_tracer/
├── shape_tracer/              # ROS 2 Python node
│   └── shape_tracer_node.py
├── config/
│   └── shapes.json            # 2D shape definitions
├── launch/
│   └── shape_tracer.launch.py # Launch RViz, MoveIt, and tracing node
├── package.xml
├── setup.py
├── setup.cfg
└── README.md
```

---

## Requirements

- ROS 2 Humble
- `xarm7_moveit_config` (from UFactory or ros-industrial)
- `moveit_commander`
- `tf_transformations`
- RViz2

Install missing Python packages:
```bash
pip install tf-transformations
```

---

## Example Shape Format (`config/shapes.json`)

```json
[
  {
    "vertices": [[0, 0], [0, 0.1], [0.1, 0.1], [0.1, 0], [0, 0]],
    "start_position": [0.05, 0.0, 0.15],
    "start_orientation_rpy": [0.0, 0.0, 0.7854]
  }
]
```

- `vertices`: List of (x, y) in meters. (0, 0) should be the first point.
- `start_position`: 3D origin of the shape in world space (meters).
- `start_orientation_rpy`: Orientation (radians) applied to the entire shape.

---

## Launch the System

```bash
ros2 launch shape_tracer shape_tracer.launch.py
```

This will:
- Launch `xarm7_moveit_config`
- Start the `shape_tracer_node` to begin tracing

---

## Customizing

To add new shapes, modify `config/shapes.json`. Each entry is traced in sequence.

If you wish to simulate the arm in Gazebo or use the real xArm, replace or extend the MoveIt execution backend in `shape_tracer_node.py`.

---

## Future Extensions

- Integration with Gazebo and robot state publishers
- Real xArm hardware control support
- Dynamic shape loading via ROS 2 service or topic

---
