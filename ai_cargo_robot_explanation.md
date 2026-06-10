# AI-Powered Cargo Transporting Robot
 
## Technical Design & Implementation Documentation
 
**Version:** 1.0  
**Platform:** Raspberry Pi 4B  
**Language:** Python 3  
**Navigation AI:** SLAM + Probabilistic Occupancy Grid + A* Path Planning  
**Localization:** Encoder Dead-Reckoning + Scan Matching + Visual Odometry  
**Drive System:** Differential Drive (L298N + 4× DC Motors)  
**Sensors:** 5× HC‑SR04 Ultrasonic Sensors + USB Camera (UVC)  
**User Interface:** Flask Web UI with Waypoints & Live Map Rendering
 
---
 
# 1. Project Overview
 
## Motivation and Scope
 
This project implements a small autonomous indoor cargo robot (~30×18×20 cm) inspired by warehouse delivery robots and robotic vacuum systems. The robot is capable of:
 
- Exploring and mapping an unknown indoor environment
- Building a reusable occupancy-grid map
- Localizing itself on the map
- Navigating from point A to point B autonomously
- Avoiding obstacles and floor drops in real time
- Operating entirely onboard without cloud compute or ROS
 
The system is designed as a classical AI rational agent rather than a neural-network-driven system. Intelligence emerges from probabilistic mapping, graph search, sensor fusion, and reactive planning.
 
## Core Requirements
 
The robot must satisfy the following capabilities simultaneously:
 
1. **Autonomous Mapping**  
   Explore an unknown flat indoor environment and generate a persistent reusable map.
 
2. **Self-Localization**  
   Estimate its own position continuously with approximately ±5 cm accuracy.
 
3. **Goal-Directed Navigation**  
   Travel to user-selected destinations using collision-free optimal paths.
 
4. **Reactive Obstacle Avoidance**  
   Detect and avoid unexpected obstacles independently of the path planner.
 
5. **Map Portability**  
   Transfer saved maps between identical robots without re-exploration.
 
## Why This Qualifies as AI
 
The system follows the classical AI agent model described in *Artificial Intelligence: A Modern Approach*:
 
| Agent Component | Implementation |
|---|---|
| Perception | Ultrasonic sensors, encoders, camera |
| World Model | Probabilistic occupancy grid |
| Planning | A* graph search |
| Action | Differential drive motor control |
 
No pre-trained neural networks are required. The project instead uses deterministic and interpretable AI techniques:
 
- SLAM (Simultaneous Localization and Mapping)
- Occupancy-grid probabilistic mapping
- Sensor fusion
- A* path planning
- Reactive safety monitoring
 
---
 
# 2. System Architecture
 
## Layered Architecture
 
The system is divided into independent layers to improve maintainability and testability.
 
| Layer | Responsibility | Key Modules |
|---|---|---|
| L1 — Hardware | GPIO, PWM, sensors, camera | `motors.py`, `sensors.py`, `camera.py` |
| L2 — Core Algorithms | SLAM, odometry, A*, safety | `slam.py`, `path_planner.py`, `odometry.py` |
| L3 — Behaviours | Exploration, navigation, idle logic | `explore.py`, `navigate.py`, `idle.py` |
| L4 — Interface | Web UI and map serving | `server.py`, `index.html`, `map.js` |
 
Each layer communicates only with adjacent layers.
 
## Operational Modes
 
### Explore Mode
 
```bash
python3 main.py --explore
```
 
- Runs SLAM and mapping
- Builds occupancy grid
- Saves `.map` file
- Streams live map to UI
 
### Navigate Mode
 
```bash
python3 main.py --map living_room --waypoint desk
```
 
- Loads saved map
- Runs A* planning
- Navigates to waypoint
- Uses reactive safety monitoring
 
### Idle Mode
 
```bash
python3 main.py --map living_room
```
 
- Loads map
- Waits for web UI commands
- Allows click-to-navigate interaction
 
---
 
# 3. Hardware Design
 
## Main Components
 
| Qty | Component | Purpose |
|---|---|---|
| 1× | Raspberry Pi 4B | Main compute platform |
| 1× | 32GB+ MicroSD | Storage |
| 4× | TT DC Gear Motors | Drive system |
| 1× | L298N Motor Driver | Motor control |
| 2× | LM393 Wheel Encoders | Odometry |
| 5× | HC-SR04 Ultrasonic Sensors | Obstacle and drop detection |
| 1× | USB Camera (UVC, 640×480) | Visual odometry + appearance detection |
| 1× | Power Bank (5V/3A) | Raspberry Pi power |
| 1× | 4× AA Battery Pack (6V) | Motor power (TT motors are rated 3–6 V) |
| 1× | Robot Chassis | Structural frame |
 
## Raspberry Pi 4B Rationale
 
The Raspberry Pi 4B was selected because it provides:
 
- Enough CPU power for SLAM + camera processing
- Hardware PWM support
- USB ports for a UVC webcam
- Full Linux + Python ecosystem
- Enough GPIO pins for sensors and motors
 
The Pi Zero and Pi Pico were rejected because they cannot reliably run concurrent SLAM, OpenCV, and a web server.
 
## Differential Drive System
 
Differential drive was selected because:
 
- It allows zero-radius turns
- Odometry mathematics are simpler
- Indoor maneuverability is significantly improved
 
Turning is achieved by varying left/right wheel speeds.
 
## LM393 Encoders
 
Wheel encoders are essential for accurate localization.
 
Without encoders:
 
- Position error accumulates rapidly
- Wheel slip becomes impossible to detect accurately
- Navigation reliability drops significantly
 
With encoders:
 
- Position error remains within practical limits
- Dead reckoning becomes viable
- Sensor fusion gains a reliable motion estimate
 
## Power Architecture
 
**Critical Rule:** Motors and Raspberry Pi must use separate power supplies.
 
### Why?
 
Motor current spikes cause:
 
- Voltage dips
- Raspberry Pi brownouts
- Random resets
- Potential hardware damage
 
### Correct Setup
 
- Power Bank → Raspberry Pi
- Battery Pack → L298N Motor Driver
- Grounds connected together only
 
---
 
# 4. Sensor Layout & Placement
 
## Sensor Placement
 
| Sensor | Position | Purpose |
|---|---|---|
| Front HC-SR04 | Front center | Forward obstacle detection |
| Right HC-SR04 | Right side | Right obstacle detection |
| Back HC-SR04 | Rear side | Reverse detection |
| Left HC-SR04 | Left side | Left obstacle detection |
| Downward HC-SR04 | Front-bottom angled down | Drop detection |
| USB Camera | Front-top | Visual odometry + appearance detection |
 
## Sensor Height Rationale
 
The four horizontal ultrasonic sensors are mounted at approximately **12 cm height** (60% of body height).
 
This placement:
 
- Detects chair and desk legs reliably
- Avoids most floor debris
- Provides stable room geometry sensing
 
## HC-SR04 Voltage Divider
 
The HC-SR04 ECHO pin outputs 5V while Raspberry Pi GPIO pins accept a maximum of 3.3V.
 
A resistor divider is mandatory:
 
```text
ECHO → 1kΩ resistor → GPIO
                |
              2kΩ
                |
               GND
```
 
Without this divider the Raspberry Pi may be permanently damaged.
 
## Drop Detection Logic
 
The downward-facing sensor normally reads approximately 15 cm.
 
Rules:
 
- Reading > 23 cm → floor disappeared → emergency stop
- Reading < 7 cm → obstacle directly below/front → stop and reverse
 
---
 
# 5. GPIO Pin Allocation
 
| Signal | GPIO (BCM) | Physical Pin |
|---|---|---|
| ENA | GPIO 12 | Pin 32 |
| IN1 | GPIO 5 | Pin 29 |
| IN2 | GPIO 6 | Pin 31 |
| ENB | GPIO 13 | Pin 33 |
| IN3 | GPIO 19 | Pin 35 |
| IN4 | GPIO 26 | Pin 37 |
| Front TRIG/ECHO | GPIO 17/27 | Pins 11/13 |
| Right TRIG/ECHO | GPIO 22/23 | Pins 15/16 |
| Back TRIG/ECHO | GPIO 24/25 | Pins 18/22 |
| Left TRIG/ECHO | GPIO 10/9 | Pins 19/21 |
| Down TRIG/ECHO | GPIO 11/8 | Pins 23/24 |
| Encoder Left | GPIO 20 | Pin 38 |
| Encoder Right | GPIO 21 | Pin 40 |
 
---
 
# 6. AI & Navigation Architecture
 
## Why Reinforcement Learning Was Rejected
 
Reinforcement Learning (RL) was considered but rejected for several reasons:
 
### 1. Poor Sample Efficiency
 
A physical robot would require thousands of training runs, making learning impractical.
 
### 2. High-Dimensional State Space
 
Continuous sensor data and camera frames would require neural-network approximation and large datasets.
 
### 3. Sim-to-Real Problems
 
Accurate simulation of wheel slip and ultrasonic behavior is extremely difficult.
 
## Why SLAM + A* Was Selected
 
SLAM + A* offers:
 
- Fast deployment without training
- Deterministic and explainable behavior
- Optimal paths on known maps
- Strong performance on structured indoor environments
 
## System Layers
 
| Layer | Responsibility |
|---|---|
| Sensing | Ultrasonic sensors + camera + encoders |
| Mapping | Occupancy-grid SLAM |
| Planning | A* path planning |
| Reacting | Safety monitoring |
| Acting | Differential drive control |
| Interface | Flask web UI |
 
---
 
# 7. Localization — Three-Source Fusion
 
## Overview
 
Localization combines three independent position estimates.
 
| Source | Purpose |
|---|---|
| Encoder Dead Reckoning | Base motion estimate |
| Scan Matching | Drift correction |
| Visual Odometry | Slip detection and motion estimation |
 
## Encoder Dead Reckoning
 
Position updates are computed from wheel movement:
 
- Left wheel displacement
- Right wheel displacement
- Robot heading change
 
Advantages:
 
- Fast
- Lightweight
- Accurate over short distances
 
Weaknesses:
 
- Wheel slip
- Accumulated drift
- Surface inconsistencies
 
## Scan Matching
 
Sensor readings are compared against the map.
 
The algorithm searches for the pose that best matches:
 
- Expected distances from map
- Actual ultrasonic readings
 
This anchors the robot to nearby walls and structures.
 
## Visual Odometry
 
OpenCV tracks feature points between frames using:
 
- Shi-Tomasi corner detection
- Lucas-Kanade optical flow
 
The camera:
 
- Detects wheel slip
- Estimates motion direction
- Improves rotational accuracy
 
The camera does **not** directly estimate room depth.
 
## Fusion Strategy
 
Weighted fusion combines all three sources.
 
| Source | Default Weight |
|---|---|
| Encoders | 0.55 |
| Scan Matching | 0.30 |
| Visual Odometry | 0.15 |
 
Weights adapt dynamically depending on environment quality and sensor confidence.
 
---
 
# 8. Mapping — Occupancy Grid Design
 
## Occupancy Grid Concept
 
The map is represented as a 2D probabilistic grid.
 
Each cell stores occupancy probability:
 
| Value | Meaning |
|---|---|
| 0 | Free |
| 50 | Unknown |
| 100 | Occupied |
 
## Bayesian Blend Update
 
Sensor readings update the grid using probabilistic blending:
 
```python
grid[cell] = current * 0.6 + evidence * 0.4
```
 
This prevents single noisy readings from corrupting the map.
 
## Ray Casting
 
For each ultrasonic reading:
 
- Cells along the beam are marked free
- End cell is marked occupied
- Unknown cells remain unchanged
 
## Grid Resolution Analysis
 
| Resolution | Accuracy | Performance |
|---|---|---|
| 10 cm/cell | Low | Very fast |
| 5 cm/cell | Good | Recommended |
| 3 cm/cell | High | Slower |
| 2 cm/cell | Very high | Overkill |
 
Recommended default:
 
- 5 cm/cell during development
- 3 cm/cell for final deployment
 
## Obstacle Inflation
 
After exploration completes, every confirmed occupied cell is expanded by approximately 10 cm using binary morphological dilation.
 
This process creates a safety margin around walls and furniture before navigation begins.
 
### Why Obstacle Inflation Is Important
 
Without obstacle inflation:
 
- The planner may attempt routes too close to furniture
- Narrow passages may appear traversable even when physically impossible
- Small localization errors could cause collisions
 
By inflating occupied regions:
 
- The robot automatically maintains safe clearance
- Path planning becomes more reliable
- Thin objects such as chair or desk legs become easier to avoid
 
The inflated map effectively treats the robot as a single point during path planning while embedding the robot's real dimensions into the map itself.
 
## Map File Format
 
The system stores maps using a compact binary `msgpack` format.
 
### Map Structure
 
```python
{
  'meta': {
    'name': str,
    'created_at': str,
    'resolution': float,
    'width_cells': int,
    'height_cells': int,
    'origin_x': int,
    'origin_y': int
  },
 
  'grid': bytes (width_cells × height_cells uint8 values, row-major),
 
  'waypoints': dict[str → (float, float)]  # name → (x_metres, y_metres)
}
```
 
## Why msgpack Was Selected
 
`msgpack` was chosen because:
 
- It is significantly smaller than JSON
- It loads faster than text formats
- It supports binary grid storage efficiently
- It can store waypoint metadata cleanly
- It is portable between robots
 
## Map Portability
 
Maps are reusable across robots running the same software stack.
 
This means:
 
- A room only needs to be explored once
- Multiple robots can share the same environment model
- Demonstrations and testing become easier
 
The system also exports a PNG representation of the occupancy grid for the web interface.
 
---
 
# 9. Path Planning — A*
 
## A* Path Planning Overview
 
The navigation system uses the A* graph-search algorithm on the occupancy grid.
 
Each traversable cell in the grid becomes a node in a graph.
 
The planner computes the optimal path between:
 
- Current robot position
- Target waypoint
 
while avoiding occupied or inflated cells.
 
## Cost Function
 
The planner evaluates each node using:
 
```text
f(n) = g(n) + h(n)
```
 
Where:
 
- `g(n)` is the known path cost from start to node `n`
- `h(n)` is the heuristic estimate to the goal
 
The heuristic uses Euclidean distance.
 
Because Euclidean distance never overestimates the true path cost, A* remains admissible and guarantees an optimal path.
 
## Walkable Cells
 
A cell is considered traversable when:
 
```text
occupancy_value < 65
```
 
This automatically excludes:
 
- Confirmed walls
- Inflated obstacles
- Unsafe narrow passages
 
## Performance
 
Typical performance on Raspberry Pi 4B:
 
| Grid Size | Search Time |
|---|---|
| 200×200 | <5 ms |
| 334×334 | <20 ms |
 
This is sufficiently fast for real-time indoor navigation.
 
## Path Smoothing
 
Raw A* paths often contain excessive zig-zag motion.
 
To improve movement quality, the system applies path smoothing.
 
### Benefits
 
- Fewer unnecessary turns
- Smoother trajectories
- Reduced motor jitter
- Lower wheel slip
- Faster navigation
 
The smoothing stage uses visibility checks between waypoints to skip intermediate nodes when possible.
 
## Route Execution
 
After planning, the robot continuously:
 
1. Computes heading error
2. Rotates toward next waypoint
3. Drives forward
4. Checks safety system
5. Updates localization estimate
6. Repeats until destination reached
 
Navigation pauses immediately if the safety system reports an obstacle.
 
---
 
# 10. Sensor Fusion & Safety Layer
 
## Safety Monitor Architecture
 
The safety system runs independently from navigation logic.
 
A dedicated daemon thread executes continuously at approximately 20 Hz.
 
This separation ensures:
 
- Navigation bugs cannot disable safety
- Emergency stopping remains responsive
- Collision handling is always active
 
## Horizontal Obstacle Detection
 
The front, left, right, and rear ultrasonic sensors monitor nearby obstacles.
 
### Stop Condition
 
If any reading drops below the configured stop threshold:
 
- Motors stop immediately
- Navigation is paused
- `blocked=True` is raised
 
For frontal obstacles, the robot may also:
 
- Reverse slightly
- Pivot away
- Retry navigation
 
## Drop Detection
 
The downward-facing ultrasonic sensor protects against:
 
- Stairs
- Floor edges
- Platform drops
 
### Trigger Conditions
 
| Condition | Action |
|---|---|
| Reading > 23 cm | Emergency stop |
| Reading < 7 cm | Stop and reverse |
 
Drop events require manual user acknowledgement before movement resumes.
 
## Camera Appearance-Based Detection
 
Ultrasonic sensors struggle with:
 
- Glass surfaces
- Very thin obstacles
- Certain reflective geometries
 
To compensate, OpenCV-based appearance analysis is added.
 
### Detection Pipeline
 
The camera system performs:
 
- Edge detection
- Contour extraction
- Lower-frame obstacle analysis
 
The camera does not directly estimate depth.
 
Instead, it acts as an advisory detection system.
 
## 3D Coverage Limitations
 
The robot primarily observes a single horizontal plane.
 
This creates unavoidable uncertainty in full 3D environments.
 
### Mitigations
 
- Obstacle inflation
- Camera-based appearance detection
- Conservative navigation margins
- Reactive stopping behavior
 
These strategies substantially reduce collision risk in practical indoor environments.
 
---
 
# 11. Software Architecture & Code Design
 
## Design Principles
 
### Single Responsibility
 
Each module performs exactly one task.
 
Examples:
 
- `motors.py` only controls motors
- `occupancy_grid.py` only manages map data
- `path_planner.py` only computes routes
 
This improves maintainability and debugging.
 
## Configuration Isolation
 
All configuration values are centralized in `config.py`.
 
This includes:
 
- GPIO pins
- Speed limits
- Thresholds
- Map resolution
- Sensor timing
 
Advantages:
 
- Easier tuning
- Cleaner code
- No magic numbers
 
## Graceful Shutdown
 
The project includes GPIO cleanup handlers.
 
On shutdown:
 
- Motors stop
- GPIO pins reset safely
- Threads terminate cleanly
 
This protects hardware from undefined states.
 
## Thread Safety
 
Several components operate concurrently:
 
- Safety monitor
- SLAM loop
- Web server
- Navigation
 
Shared resources are protected using synchronization locks.
 
## Testability
 
Core algorithms are hardware-independent whenever possible.
 
This allows:
 
- Development on non-Pi systems
- Faster debugging
- Unit testing without sensors
 
---
 
## Project Structure
 
```text
robot_car/
├── hardware/
│   ├── motors.py
│   ├── sensors.py
│   ├── camera.py
│   └── gpio_cleanup.py
│
├── core/
│   ├── occupancy_grid.py
│   ├── slam.py
│   ├── path_planner.py
│   ├── safety_monitor.py
│   └── odometry.py
│
├── modes/
│   ├── explore.py
│   ├── navigate.py
│   └── idle.py
│
├── ui/
│   ├── server.py
│   └── static/
│
├── maps/
├── config.py
├── main.py
└── requirements.txt
```
 
## Main Python Dependencies
 
| Library | Purpose |
|---|---|
| RPi.GPIO | GPIO control |
| pigpio | PWM motor control |
| OpenCV | Visual odometry + detection |
| NumPy | Grid operations |
| SciPy | Obstacle inflation |
| Flask | Web UI |
| Flask-SocketIO | Live updates |
| Pillow | PNG export |
| msgpack | Map serialization |
 
---
 
# 12. Web UI Architecture
 
## Flask Web Interface
 
The robot exposes a lightweight Flask-based web application.
 
The UI allows users to:
 
- View the occupancy grid
- Monitor robot position
- Select destinations
- Manage waypoints
 
## Main Endpoints
 
| Endpoint | Purpose |
|---|---|
| `/` | Main interface |
| `/map.png` | Current occupancy grid |
| `/navigate` | Navigation commands |
| `/socket.io` | Live updates |
 
## Live Updates
 
The browser receives:
 
- Position updates
- Path overlays
- Map refreshes
 
using WebSocket communication.
 
The UI intentionally remains minimal and functional rather than visually complex.
 
---
 
# 13. Development Phases
 
## Phase 1 — Hardware & Motor Control
 
Goals:
 
- Drive motors successfully
- Read all sensors
- Validate GPIO control
 
## Phase 2 — Encoder Odometry
 
Goals:
 
- Track robot displacement
- Compute heading changes
- Validate localization accuracy
 
## Phase 3 — Safety Layer
 
Goals:
 
- Reliable obstacle stopping
- Working drop detection
- Independent safety thread
 
## Phase 4 — SLAM Mapping
 
Goals:
 
- Autonomous exploration
- Occupancy-grid generation
- Persistent `.map` creation
 
This is the first major AI integration phase.
 
## Phase 5 — A* Navigation
 
Goals:
 
- Waypoint routing
- Autonomous point-to-point travel
- Reliable path execution
 
## Phase 6 — Web Interface
 
Goals:
 
- Click-to-navigate support
- Live visualization
- Waypoint management
 
## Phase 7 — Camera Integration
 
Goals:
 
- Visual odometry
- Appearance-based detection
- Improved localization accuracy
 
## Minimum Viable Demonstration
 
A successful demonstration includes:
 
- Room exploration
- Map saving
- Autonomous navigation
- Obstacle avoidance
- Waypoint selection
 
---
 
# 14. Key Design Decisions
 
| Decision | Selected Approach | Reason |
|---|---|---|
| Navigation AI | SLAM + A* | More practical and explainable than RL |
| Map Representation | Occupancy Grid | Robust against noisy sensor data |
| Camera Depth | Not used | Monocular camera lacks depth information |
| Sensor Height | 12 cm | Reliable furniture detection |
| Obstacle Inflation | 10 cm | Safer navigation |
| Sensor Count | 5 ultrasonic sensors | Good balance of coverage and complexity |
| User Destination Input | Web UI | Simple and flexible |
| Power System | Separate supplies | Prevents Raspberry Pi resets |
| Map Storage | msgpack | Compact and portable |
 
---
 
# 15. Known Limitations
 
## 2D Sensing in a 3D World
 
The robot observes primarily one horizontal plane.
 
### Consequences
 
- Some overhanging obstacles may be missed
- Thin objects remain difficult to detect
 
### Mitigations
 
- Obstacle inflation
- Camera appearance analysis
- Conservative routing
 
## Glass Detection Issues
 
Ultrasonic waves reflect poorly from glass surfaces.
 
Mitigation:
 
- Edge detection using camera frames
 
## Map Drift
 
Long-distance navigation accumulates localization error.
 
Mitigations:
 
- Scan matching
- Visual odometry
- Conservative map resolution
 
## No Dynamic Obstacle Tracking
 
The occupancy grid is static during navigation.
 
Moving obstacles are handled reactively rather than inserted into the map.
 
## No Multi-Floor Support
 
The robot is intentionally designed for flat indoor environments only.
 
## No Loop Closure
 
The SLAM implementation does not explicitly detect revisiting the same location.
 
This is acceptable for:
 
- Single-room environments
- Small indoor spaces
- Controlled demonstrations
 
---
 
# 16. Conclusion
 
The AI-Powered Cargo Transporting Robot demonstrates a complete classical robotics architecture running entirely on embedded hardware.
 
The project combines:
 
- Probabilistic SLAM
- Multi-source localization
- Occupancy-grid mapping
- A* navigation
- Reactive safety systems
- Web-based interaction
 
without requiring:
 
- Cloud computation
- Neural networks
- ROS middleware
- External infrastructure
 
The design emphasizes:
 
- Interpretability
- Modularity
- Reliability
- Practical engineering
 
This project serves as a strong demonstration of autonomous robotics and classical artificial intelligence principles in a real-world embedded environment.
 
---
 
**End of Documentation — AI-Powered Cargo Transporting Robot v1.0**