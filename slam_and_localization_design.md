# SLAM & Localization — Design Decisions
## AI Cargo Robot — Discussion Notes

---

## 1. What is SLAM

SLAM (Simultaneous Localization and Mapping) solves a chicken-and-egg problem:
- To build a map, you need to know where you are
- To know where you are, you need a map

SLAM solves both simultaneously and incrementally. The robot starts with no map and no known position, moves around, and builds both together in real time.

---

## 2. SLAM vs Localization — Relationship

| Mode | What runs |
|---|---|
| **Explore mode** | Full SLAM — mapping + localization together |
| **Navigate mode** | Localization only — map is fixed, robot tracks its position on it |

Localization is a *component* of SLAM during exploration, and a *standalone* process during navigation. They share the same underlying algorithms.

---

## 3. Occupancy Grid

### Concept

The map is a 2D grid where every cell stores the probability that the cell is occupied:

| Value | Meaning |
|---|---|
| 0 | Definitely free |
| 50 | Unknown (initial state) |
| 100 | Definitely occupied |

Each cell represents a 5 cm × 5 cm square of real floor space.

### Decision: NumPy uint8 array

```python
import numpy as np
grid = np.full((height, width), 50, dtype=np.uint8)
```

**Reason:** Ray casting and dilation operate on thousands of cells simultaneously. NumPy vectorized operations are orders of magnitude faster than Python loops. Already a core dependency.

### Grid Resolution

| Resolution | Accuracy | Performance |
|---|---|---|
| 10 cm/cell | Low | Very fast |
| 5 cm/cell | Good | Recommended for development |
| 3 cm/cell | High | Recommended for final deployment |
| 2 cm/cell | Very high | Overkill |

---

## 4. Ray Casting

### Concept

Every ultrasonic sensor reading is projected as a ray from the robot's current position:
- All cells the ray passes **through** → marked more free
- The cell at the **end** of the ray → marked more occupied

### Decision: Bresenham's Line Algorithm

**Reason:** Standard algorithm for grid-based ray casting. Integer-only operations (no floating point), fast, well understood, and sufficient at 5 sensors × 20 Hz on a Pi 4B. DDA and NumPy vectorized approaches offer marginal gains not worth the added complexity.

```python
def bresenham(x0, y0, x1, y1):
    # Returns list of (x, y) grid cells along the line
    cells = []
    dx = abs(x1 - x0)
    dy = abs(y1 - y0)
    sx = 1 if x0 < x1 else -1
    sy = 1 if y0 < y1 else -1
    err = dx - dy
    while True:
        cells.append((x0, y0))
        if x0 == x1 and y0 == y1:
            break
        e2 = 2 * err
        if e2 > -dy:
            err -= dy
            x0 += sx
        if e2 < dx:
            err += dx
            y0 += sy
    return cells
```

---

## 5. Bayesian Cell Update — Log-Odds

### Concept

Rather than directly writing 0 or 100 into a cell (which would make the map fragile to noisy readings), each new sensor reading is blended probabilistically into the existing belief.

### Decision: Log-Odds Update

**Reason:** Mathematically correct probabilistic inference. Handles extreme probabilities better than a linear blend. Prevents cells from getting permanently locked at certainty from a single reading. Important for academic justification of the AI approach.

```python
import math

# Convert probability to log-odds
def prob_to_log_odds(p):
    return math.log(p / (1 - p))

# Update cell with new evidence probability.
# Convention: cells are addressed as (col, row); the numpy array is row-major,
# so it is always indexed [row, col] (never [x, y]). See feature_plan.md.
def update_cell(log_odds_grid, col, row, p_evidence):
    log_odds_grid[row, col] += prob_to_log_odds(p_evidence)
    # Clamp to prevent overflow
    log_odds_grid[row, col] = max(-10, min(10, log_odds_grid[row, col]))

# Convert back to probability for display
def log_odds_to_prob(lo):
    return 1 - (1 / (1 + math.exp(lo)))
```

Typical evidence values:
- Free cell along ray: p = 0.3 (nudges toward free)
- Occupied endpoint: p = 0.7 (nudges toward occupied)
- No reading: leave unchanged

---

## 6. Obstacle Inflation

### Concept

After exploration completes, every confirmed occupied cell is expanded outward by ~10 cm. This embeds the robot's physical dimensions into the map so A* can treat the robot as a point during planning while still maintaining safe clearance from obstacles.

### Decision: SciPy Binary Dilation

**Reason:** One line of code, well-tested, fast, already a dependency for other grid operations.

```python
from scipy.ndimage import binary_dilation
import numpy as np

occupied_mask = grid > 65  # threshold for confirmed occupied
inflated_mask = binary_dilation(occupied_mask, iterations=2)  # 2 cells × 5 cm = 10 cm
grid[inflated_mask] = 100
```

### Why Inflation Matters

Without inflation:
- Planner routes robot 1 cm from a wall — fine on paper, collision in reality
- Localization error alone can cause a hit
- Thin objects like chair legs nearly invisible to planner

With inflation:
- Robot automatically maintains safe clearance
- Path planning becomes more reliable
- Thin obstacles become effectively wider in the map

---

## 7. Exploration Strategy

### Decision: Frontier-Based Exploration

**Reason:** Guarantees complete coverage of all reachable space. Handles arbitrary room shapes and furniture layouts. Standard approach in robotics research. Random walk and wall following are significantly less reliable.

### How It Works

1. Scan the grid for **frontier cells** — cells that are known-free but adjacent to unknown cells
2. Group nearby frontier cells into frontier regions
3. Navigate to the centroid of the nearest frontier region
4. As the robot arrives, new frontiers are discovered
5. Repeat until no frontier cells remain → map is complete

```python
from collections import deque

def find_frontiers(grid):
    frontiers = []
    for y in range(grid.shape[0]):
        for x in range(grid.shape[1]):
            if is_free(grid, x, y) and has_unknown_neighbor(grid, x, y):
                frontiers.append((x, y))
    return frontiers
```

---

## 8. Scan Matching

### Concept

Corrects the robot's pose estimate by comparing current sensor readings against the existing map. Asks: "given what the sensors currently read, what position would make those readings best match the map?"

### Decision: Correlation Window Search

**Reason:** Computationally affordable on Pi 4B. Robust enough for small localization corrections in indoor environments. ICP is overkill for 5 ultrasonic sensors. Gradient search risks local minima.

### How It Works

1. Take current sensor readings
2. Search a small window around current pose estimate (e.g. ±15 cm, ±10°)
3. For each candidate pose, score it by how well sensor readings match the map
4. Select the highest-scoring candidate as the corrected pose
5. Only accept correction if score exceeds a confidence threshold

```python
def scan_match(grid, pose, sensor_readings, search_range=0.15, angle_range=0.17):
    best_score = -1
    best_pose = pose
    for dx in np.arange(-search_range, search_range, 0.05):
        for dy in np.arange(-search_range, search_range, 0.05):
            for dtheta in np.arange(-angle_range, angle_range, 0.05):
                candidate = (pose[0]+dx, pose[1]+dy, pose[2]+dtheta)
                score = score_pose(grid, candidate, sensor_readings)
                if score > best_score:
                    best_score = score
                    best_pose = candidate
    return best_pose, best_score
```

---

## 9. Localization — Three-Source Fusion

### Source 1: Encoder Dead Reckoning

Base layer. Every movement updates position using wheel pulse counts.

**Algorithm:** Differential drive kinematics

```python
delta_left  = left_pulses  * dist_per_pulse
delta_right = right_pulses * dist_per_pulse

delta_center = (delta_left + delta_right) / 2
delta_theta  = (delta_right - delta_left) / wheel_base

x     += delta_center * cos(theta)
y     += delta_center * sin(theta)
theta += delta_theta
```

**Strengths:** Fast, lightweight, reliable over short distances, always available.
**Weaknesses:** Accumulates drift from wheel slip, uneven floor, motor imbalance.
**Default weight:** 0.55

---

### Source 2: Scan Matching

Uses the correlation window search described above to anchor the pose estimate to nearby walls and structures.

**Strengths:** Corrects accumulated encoder drift, anchors robot to map geometry.
**Weaknesses:** Unreliable in featureless areas (long open corridors). Computationally more expensive.
**Default weight:** 0.30
**Dynamic:** Weight drops toward 0 when match confidence score is below threshold.

---

### Source 3: Visual Odometry

The USB camera tracks how the visual scene shifts between frames to estimate robot motion.

**Algorithm:** Shi-Tomasi corner detection + Lucas-Kanade optical flow

**Reason for this choice over ORB:** Significantly lighter computationally. ORB feature matching is more robust but too heavy to run alongside SLAM and Flask on a Pi 4B. Shi-Tomasi + LK is the standard lightweight choice and sufficient for slip detection.

```python
import cv2

# Detect features in current frame
prev_gray = cv2.cvtColor(prev_frame, cv2.COLOR_BGR2GRAY)
features = cv2.goodFeaturesToTrack(prev_gray, maxCorners=100,
                                    qualityLevel=0.3, minDistance=7)

# Track features in next frame
curr_gray = cv2.cvtColor(curr_frame, cv2.COLOR_BGR2GRAY)
next_pts, status, _ = cv2.calcOpticalFlowPyrLK(prev_gray, curr_gray,
                                                features, None)

# Good tracked points
good_prev = features[status == 1]
good_next = next_pts[status == 1]
```

**Primary uses:**
- Detect wheel slip (wheels report motion, camera reports none)
- Improve rotational accuracy
- Provide motion estimate in encoder-degraded conditions

**Important limitation:** Monocular camera gives no depth information. Visual odometry contributes 2D motion estimation only.

**Default weight:** 0.15
**Dynamic:** Weight drops when lighting is poor, motion is too fast, or too few features are tracked.

---

### Fusion Strategy

**Decision: Dynamic weight adjustment**

Each source produces a confidence score alongside its pose estimate. Weights are normalised from confidence scores at every fusion step.

**Reason over fixed weights:** Fixed weights perform poorly in degraded conditions — poor lighting kills visual odometry, featureless corridors kill scan matching. Dynamic weighting lets the system gracefully degrade rather than fail.

```python
def fuse_poses(encoder_pose, scan_pose, visual_pose,
               encoder_conf, scan_conf, visual_conf):
    total = encoder_conf + scan_conf + visual_conf
    w_enc = encoder_conf / total
    w_scn = scan_conf   / total
    w_vis = visual_conf / total

    x     = w_enc*encoder_pose[0] + w_scn*scan_pose[0] + w_vis*visual_pose[0]
    y     = w_enc*encoder_pose[1] + w_scn*scan_pose[1] + w_vis*visual_pose[1]
    theta = w_enc*encoder_pose[2] + w_scn*scan_pose[2] + w_vis*visual_pose[2]
    return (x, y, theta)
```

---

## 10. Full Decision Summary

### SLAM

| Component | Decision | Reason |
|---|---|---|
| Grid storage | NumPy uint8 array | Fast vectorized operations |
| Ray casting | Bresenham's algorithm | Standard, fast, integer-only |
| Cell update | Log-odds Bayesian | Mathematically correct, drift-resistant |
| Obstacle inflation | SciPy binary dilation | One line, already a dependency |
| Exploration | Frontier-based | Guaranteed complete coverage |
| Scan matching | Correlation window search | Affordable on Pi 4B |

### Localization

| Source | Algorithm | Default Weight | Dynamic? |
|---|---|---|---|
| Encoder dead reckoning | Differential drive kinematics | 0.55 | No |
| Scan matching | Correlation window search | 0.30 | Yes — drops in featureless areas |
| Visual odometry | Shi-Tomasi + Lucas-Kanade | 0.15 | Yes — drops in poor lighting/motion |

---

*Notes from design discussion — AI Cargo Robot project*
