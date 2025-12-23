# Obstacle Avoidance Koch
A Sim-to-Real framework for **real-time dynamic obstacle avoidance** using the Koch v1.1 robotic arm.To enable the robot to react to dynamic obstacles in the physical world with low latency.

## ✨ Features

- [x] **Vision-based Perception**: Real-time object detection and tracking pipeline powered by **OpenCV**.
- [x] **Real-time Reaction**: High-frequency control loop ensuring immediate response to dynamic obstacles.
- [x] **Sim-to-Real Transfer**: Validated in PyBullet simulation and deployed on physical hardware.
- [x] **Dynamic Safety**: Utilizes implicit neural representations (or potential fields) for smooth collision avoidance.

### Step 1: Launch the Hardware Controller
Open the first terminal to start the low-level driver that communicates with the physical Koch arm.

```bash
# Navigate to the package directory
cd koch_ros

# Start the real arm controller (Zero position)
python3 koch_real_arm_controller_zero.py
```

### Step 2: Execute the Obstacle Avoidance Loop
Open a second terminal, navigate to the project root, and run the main program. This script handles the Sim-to-Real logic and OpenCV perception.
```bash
# Run the main control loop with real obstacles
python3 koch_mirror_with_real_obstacles.py
```
