# A* Path Planning — Design Decisions
## AI Cargo Robot — Discussion Notes

---

## 1. What A* Is

A* is a graph search algorithm that finds the shortest path between two points by exploring nodes in order of their estimated total cost — not just how far you have already travelled, but how far you still have to go.

It is an upgrade of Dijkstra's algorithm. Dijkstra explores in all directions equally. A* is guided toward the goal by a heuristic, making it significantly faster in practice.

---

## 2. Grid to Graph Conversion

The occupancy grid maps directly to a graph:
- Every traversable cell (occupancy < 65) becomes a node
- Adjacent traversable cells are connected by edges

### Movement Directions

**Decision: 8-directional (includes diagonals)**

| Option | Turns | Path Quality | Complexity |
|---|---|---|---|
| 4-directional | Many 90° turns | Staircase-like, unnatural | Simpler |
| 8-directional | Fewer turns | Natural, smooth | Minor added complexity |

Reason: More natural movement patterns, fewer unnecessary turns, better suited to how a differential drive robot actually moves. Diagonal edges must be weighted correctly at √2 × cell size rather than 1 × cell size.

---

## 3. Cost Function

Every node is evaluated with:

```
f(n) = g(n) + h(n)
```

- `g(n)` — actual known cost from start to node n (accumulated during search)
- `h(n)` — estimated cost from node n to goal (heuristic)

The algorithm always expands the node with the lowest `f(n)` first using a priority queue.

---

## 4. Heuristic

The heuristic must never overestimate the true cost. If it does, A* loses its optimality guarantee (admissibility).

### Decision: Euclidean Distance

```python
import math

def heuristic(a, b):
    return math.sqrt((a[0] - b[0])**2 + (a[1] - b[1])**2)
```

| Option | Formula | Admissible for 8-dir? | Notes |
|---|---|---|---|
| Manhattan | `|dx| + |dy|` | No — overestimates diagonals | Only valid for 4-directional |
| Euclidean | `sqrt(dx² + dy²)` | Yes | Correct for 8-directional |
| Chebyshev | `max(|dx|, |dy|)` | Yes | Faster but explores more nodes |

Reason: We are using 8-directional movement with correctly weighted diagonal costs. Euclidean is the correct admissible heuristic. The sqrt computation is negligible on a Pi 4B at indoor map scales.

---

## 5. Data Structures

### Open Set — Nodes Discovered, Not Yet Evaluated

**Decision: Python `heapq` min-heap**

```python
import heapq

open_set = []
heapq.heappush(open_set, (f_score, node))
current = heapq.heappop(open_set)
```

Reason: O(log n) insertion and extraction. No extra dependencies. Standard choice for A* implementation.

### Closed Set — Nodes Already Evaluated

**Decision: NumPy boolean array**

```python
import numpy as np

closed_set = np.zeros((grid_height, grid_width), dtype=bool)
closed_set[y, x] = True
```

Reason: O(1) lookup, memory efficient for large grids, avoids Python set overhead.

---

## 6. Walkable Cell Threshold

```python
def is_walkable(grid, x, y):
    return grid[y, x] < 65
```

This excludes:
- Confirmed walls (value 100)
- Inflated obstacle regions (value 100)
- Borderline uncertain cells near obstacles

Free cells (0–50) and lightly uncertain cells pass through safely.

---

## 7. Core A* Implementation Sketch

```python
import heapq
import numpy as np
import math

def astar(grid, start, goal):
    open_set = []
    heapq.heappush(open_set, (0, start))

    came_from = {}
    g_score = {start: 0}
    closed_set = np.zeros(grid.shape, dtype=bool)

    directions = [
        (0,1,1), (1,0,1), (0,-1,1), (-1,0,1),       # cardinal: cost 1
        (1,1,1.414), (1,-1,1.414), (-1,1,1.414), (-1,-1,1.414)  # diagonal: cost √2
    ]

    while open_set:
        _, current = heapq.heappop(open_set)

        if current == goal:
            return reconstruct_path(came_from, current)

        if closed_set[current[1], current[0]]:
            continue
        closed_set[current[1], current[0]] = True

        for dx, dy, cost in directions:
            neighbor = (current[0]+dx, current[1]+dy)
            if not in_bounds(grid, neighbor):
                continue
            if not is_walkable(grid, *neighbor):
                continue
            if closed_set[neighbor[1], neighbor[0]]:
                continue

            tentative_g = g_score[current] + cost

            if tentative_g < g_score.get(neighbor, float('inf')):
                came_from[neighbor] = current
                g_score[neighbor] = tentative_g
                f = tentative_g + heuristic(neighbor, goal)
                heapq.heappush(open_set, (f, neighbor))

    return None  # No path found

def reconstruct_path(came_from, current):
    path = [current]
    while current in came_from:
        current = came_from[current]
        path.append(current)
    return path[::-1]
```

---

## 8. Path Smoothing — Visibility-Based Pruning

### Problem

Raw A* on a grid produces zigzag paths even along what should be straight corridors. This causes excessive turning, motor jitter, slower navigation, and more localization error.

### Decision: Visibility-Based Pruning (String Pulling)

**Reason:** Simple, guarantees obstacle clearance by checking actual grid cells, produces clean straight-line segments. Bezier/spline smoothing is overkill for a differential drive robot and harder to guarantee clearance along curves.

### How It Works

1. Start at the first waypoint
2. Try to draw a straight line to the waypoint two steps ahead
3. If the line passes only through free cells → remove the intermediate waypoint
4. Advance and repeat until no more waypoints can be removed

```python
def smooth_path(path, grid):
    if len(path) <= 2:
        return path

    smoothed = [path[0]]
    current_idx = 0

    while current_idx < len(path) - 1:
        # Try to reach as far ahead as possible in a straight line
        furthest = current_idx + 1
        for lookahead in range(current_idx + 2, len(path)):
            if line_of_sight(grid, path[current_idx], path[lookahead]):
                furthest = lookahead
        smoothed.append(path[furthest])
        current_idx = furthest

    return smoothed

def line_of_sight(grid, a, b):
    # Use Bresenham to get all cells between a and b
    cells = bresenham(a[0], a[1], b[0], b[1])
    return all(is_walkable(grid, x, y) for x, y in cells)
```

---

## 9. Replanning Strategy

### What Triggers Replanning

The safety monitor stops the robot when an unexpected obstacle is detected. The navigation system must decide what to do next.

### Decision: Wait-and-Retry, Then Full Replan

| Attempt | Action |
|---|---|
| 1–3 | Wait briefly, retry same path (handles people walking past) |
| 4+ | Trigger full A* replan from current position to original goal |

**Reason:** A full replan from current position takes < 20ms on a Pi 4B for typical indoor maps — cheap enough to do freely. D* Lite (incremental replanning) adds significant complexity for no meaningful performance gain at this scale.

```python
def navigate_to_goal(goal):
    retry_count = 0
    path = astar(grid, current_pose, goal)

    while not at_goal():
        if safety_monitor.blocked:
            retry_count += 1
            if retry_count >= 3:
                path = astar(grid, current_pose, goal)  # full replan
                retry_count = 0
            time.sleep(0.5)
            continue

        follow_next_waypoint(path)
        retry_count = 0
```

---

## 10. Route Execution

After planning, the robot executes the smoothed path waypoint by waypoint:

```
1. Compute heading error to next waypoint
2. Rotate in place to align heading
3. Drive forward
4. Check safety system continuously
5. Update pose estimate (localization)
6. If within arrival threshold → advance to next waypoint
7. Repeat until goal reached
```

### Arrival Threshold

**Decision: 8–10 cm (roughly 2 grid cells)**

Too tight → robot oscillates trying to hit an exact cell centre
Too loose → corners get cut, risk brushing obstacles

8–10 cm balances smooth waypoint transitions with accurate navigation.

---

## 11. Performance on Raspberry Pi 4B

| Grid size | Typical search time |
|---|---|
| 200 × 200 cells | < 5 ms |
| 334 × 334 cells | < 20 ms |

Fast enough for real-time replanning without impacting SLAM, safety, or web server threads.

---

## 12. Decision Summary

| Component | Decision | Reason |
|---|---|---|
| Movement directions | 8-directional | Natural paths, fewer turns |
| Heuristic | Euclidean distance | Admissible for 8-directional movement |
| Open set | `heapq` min-heap | O(log n), no extra dependencies |
| Closed set | NumPy boolean array | O(1) lookup, memory efficient |
| Walkable threshold | occupancy < 65 | Excludes walls and inflated regions |
| Path smoothing | Visibility-based pruning | Simple, obstacle-safe, straight segments |
| Replanning | Wait-retry then full replan | Handles temporary and permanent obstacles |
| Arrival threshold | 8–10 cm | Smooth waypoint transitions |

---

*Notes from design discussion — AI Cargo Robot project*
