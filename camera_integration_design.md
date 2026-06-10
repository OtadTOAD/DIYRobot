# Camera Integration & Visual Odometry — Design Decisions
## AI Cargo Robot — Discussion Notes

---

## 1. Camera Hardware

### Decision: USB Camera via USB-A port

Rather than the Raspberry Pi Camera Module v2 (CSI interface), the project uses a standard USB camera connected via USB-A.

**Practical implications:**

| Aspect | Detail |
|---|---|
| Library | Pure OpenCV `VideoCapture` — no `picamera2` needed |
| Resolution | Set explicitly to 640×480 |
| Frame rate | Target 20–30 fps |
| Driver | Linux UVC driver — automatic, no configuration |
| Verify on boot | `/dev/video0` should appear when camera is plugged in |

**Why 640×480 and not higher?**
Higher resolution wastes CPU cycles on the Pi for no meaningful benefit. Optical flow tracking and edge detection work well at this resolution. 1080p would slow every pipeline stage with no improvement in output quality.

```python
cap = cv2.VideoCapture(0)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
cap.set(cv2.CAP_PROP_FPS, 30)
```

---

## 2. What the Camera Does — Two Distinct Jobs

The camera serves two completely separate pipelines running on the same frame stream:

| Pipeline | Job | Output |
|---|---|---|
| Visual Odometry | Motion estimation between frames | Pose delta + confidence score → localization fusion |
| Appearance Detection | Obstacle detection for sonar blind spots | Advisory flag → safety monitor |

These are kept conceptually and architecturally separate. The same thread captures frames and feeds both pipelines.

---

## 3. Pipeline 1 — Visual Odometry

### Role in the System

The camera answers one question: **"how did the robot move between this frame and the last one?"**

It does not need to know about the map. It produces a motion estimate (dx, dy, dtheta) and a confidence score. The SLAM localization fusion system consumes these alongside encoder dead reckoning and scan matching, with a default weight of 0.15 that adjusts dynamically.

### Algorithm: Shi-Tomasi Corner Detection + Lucas-Kanade Optical Flow

**Reason for this choice over ORB or SIFT:**
Significantly lighter computationally. ORB and SIFT are more robust in challenging conditions but too CPU-heavy to run alongside SLAM, safety monitoring, and Flask on a Pi 4B. Shi-Tomasi + LK is the standard lightweight choice for embedded visual odometry.

---

### Step 1 — Feature Detection

Shi-Tomasi finds strong corner features — visually distinctive points reliable enough to track across frames.

Detection does not run every frame. Detect once, track for several frames, re-detect when tracked features drop below 30 good points.

```python
gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
features = cv2.goodFeaturesToTrack(
    gray,
    maxCorners=150,
    qualityLevel=0.01,
    minDistance=10,
    blockSize=7
)
```

---

### Step 2 — Optical Flow Tracking (Lucas-Kanade Pyramidal)

Tracks detected feature points from the previous frame into the current frame.

`maxLevel=3` enables a 3-level image pyramid — this handles larger inter-frame motions without losing track of fast-moving features.

```python
next_pts, status, error = cv2.calcOpticalFlowPyrLK(
    prev_gray, curr_gray,
    prev_features, None,
    winSize=(21, 21),
    maxLevel=3,
    criteria=(cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 30, 0.01)
)
good_prev = prev_features[status == 1]
good_next = next_pts[status == 1]
```

---

### Step 3 — Motion Estimation

Aggregate movement of all tracked feature points encodes robot motion:
- Average horizontal flow → translational motion estimate
- Rotational flow pattern → angular motion estimate
- Convert pixel flow to physical units using known camera mounting height and focal length

```python
if len(good_prev) > 10:
    flow = good_next - good_prev
    avg_flow_x = np.mean(flow[:, 0])
    avg_flow_y = np.mean(flow[:, 1])
    # Convert pixel flow → physical displacement
    # using camera intrinsic parameters calibrated at setup
```

---

### Step 4 — Confidence Scoring

Number of successfully tracked features and their motion consistency produces a confidence score. Inconsistent flow (points moving in random directions) signals poor quality — the fusion system reduces this source's weight automatically.

```python
def flow_confidence(good_prev, good_next, status):
    n_tracked = np.sum(status)
    if n_tracked < 10:
        return 0.0
    flow = good_next - good_prev
    consistency = 1.0 - (np.std(flow) / (np.mean(np.abs(flow)) + 1e-5))
    return float(np.clip(n_tracked / 150 * consistency, 0, 1))
```

Confidence drops toward 0 when:
- Too few features tracked (low-texture surfaces, poor lighting)
- Motion between frames is too fast (camera blur)
- Feature motion is inconsistent (camera shake)

---

### Wheel Slip Detection

This is one of the most valuable uses of the camera. If encoders report forward motion but the camera sees no scene change, wheel slip is occurring.

```python
encoder_delta = compute_encoder_displacement()
visual_delta  = compute_visual_displacement()

if encoder_delta > SLIP_THRESHOLD and visual_delta < SLIP_THRESHOLD * 0.3:
    declare_wheel_slip()
    reduce_encoder_confidence()
    log_slip_event()
```

When slip is detected:
- Encoder confidence weight is temporarily reduced
- Visual odometry and scan matching weights increase proportionally
- Slip event is logged to the status log

---

## 4. Pipeline 2 — Appearance-Based Obstacle Detection

### Why It's Needed

Ultrasonic sensors have blind spots:
- Glass surfaces — sound passes through or reflects at wrong angle
- Very thin vertical obstacles — legs thinner than ~2 cm
- Objects positioned at angles that deflect the ultrasonic beam

The camera catches what sonar misses.

### Decision: Advisory only — not authoritative

The camera does not trigger hard stops on its own. It raises a soft advisory flag that reduces speed and increases sonar sensitivity. Sole reliance on camera for stopping would cause false positives (shadows, patterns on floor) that would make the robot unreliable.

---

### Algorithm: Region of Interest + Canny Edge Detection + Contour Analysis

**Step 1 — Region of Interest**

Only the bottom 45% of the frame is analysed. Floor-level obstacles appear in the lower portion of the frame given the camera's forward-facing mounting position. Analysing the full frame wastes CPU.

```python
h, w = frame.shape[:2]
roi = frame[int(h * 0.55):h, :]
```

**Step 2 — Edge Detection**

```python
gray_roi = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
blurred  = cv2.GaussianBlur(gray_roi, (5, 5), 0)
edges    = cv2.Canny(blurred, threshold1=50, threshold2=150)
```

**Step 3 — Contour Analysis**

```python
contours, _ = cv2.findContours(
    edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
)
significant = [c for c in contours if cv2.contourArea(c) > MIN_CONTOUR_AREA]
```

**Step 4 — Advisory Signal**

If significant contours appear in the lower-centre region and sonar does not confirm an obstacle, raise a soft advisory flag:
- Robot reduces speed
- Safety monitor sonar thresholds tighten
- No hard stop

---

## 5. Threading Architecture

The camera runs as an independent daemon thread. It does not block SLAM, safety, or navigation.

### Full Thread Model

```
Thread 1 — Safety monitor        (~20 Hz)
  reads:  sonar sensors, camera advisory flag
  writes: blocked flag, emergency stop signal

Thread 2 — Camera processor      (~20–30 Hz)
  reads:  USB camera frames
  writes: visual odometry estimate + confidence,
          camera advisory flag

Thread 3 — SLAM / Localization   (~10 Hz)
  reads:  encoders, sonar, visual odometry estimate + confidence
  writes: occupancy grid, robot pose estimate

Thread 4 — Navigation            (event-driven)
  reads:  robot pose, occupancy grid, blocked flag
  writes: motor commands

Thread 5 — Flask web server      (event-driven)
  reads:  robot pose, map, mode
  writes: UI state
```

### Shared State and Locks

| Shared resource | Written by | Read by | Lock type |
|---|---|---|---|
| `visual_odometry_estimate` | Camera thread | SLAM thread | `threading.Lock` |
| `camera_advisory_flag` | Camera thread | Safety thread | `threading.Lock` |
| `robot_pose` | SLAM thread | Navigation, Web | `threading.RLock` |
| `occupancy_grid` | SLAM thread | Navigation, Web | `threading.RLock` |
| `blocked_flag` | Safety thread | Navigation | `threading.Lock` |

`RLock` (reentrant lock) is used for pose and grid because the SLAM thread may need to read its own data while in the middle of a write cycle.

### Camera → SLAM Data Handoff

The camera thread runs faster than SLAM. They don't block each other — the camera writes its latest estimate under a lock, SLAM reads it at its own pace. Missing a frame is acceptable; the most recent estimate is always used.

```python
# Camera thread — writes at ~25 Hz
with vo_lock:
    vo_estimate   = (dx, dy, dtheta)
    vo_confidence = score

# SLAM thread — reads at ~10 Hz
with vo_lock:
    est  = vo_estimate
    conf = vo_confidence
# Uses est and conf in weighted fusion
```

---

## 6. Priority Hierarchy — Who Has Final Say

When systems produce conflicting signals, a strict priority hierarchy applies:

```
1. Safety monitor        HIGHEST — emergency stop overrides everything, no exceptions
2. Camera advisory       SECOND  — reduces speed, tightens sonar thresholds, no hard stop
3. SLAM localization     THIRD   — corrects pose, may trigger route replan
4. Navigation            LOWEST  — executes only when all above give clearance
```

This means:
- Navigation never fights safety
- Safety never needs to know about the navigation plan
- Each layer is independent and communicates only through well-defined shared flags and locks

---

## 7. Tech Stack

All camera functionality is implemented using existing dependencies — no new libraries required.

| Component | Library | Notes |
|---|---|---|
| Camera capture | `cv2.VideoCapture` | USB camera, UVC driver |
| Feature detection | `cv2.goodFeaturesToTrack` | Shi-Tomasi, built into OpenCV |
| Optical flow | `cv2.calcOpticalFlowPyrLK` | Lucas-Kanade pyramidal, built into OpenCV |
| Edge detection | `cv2.Canny` | Built into OpenCV |
| Contour analysis | `cv2.findContours` | Built into OpenCV |
| Numerical ops | `numpy` | Flow vector maths, confidence scoring |
| Threading | `threading` | Camera as daemon thread |

---

## 8. Decision Summary

| Decision | Choice | Reason |
|---|---|---|
| Camera type | USB camera via USB-A | Cheaper, better resolution, simpler driver |
| Capture library | `cv2.VideoCapture` | No `picamera2` needed |
| Resolution | 640×480 | Sufficient for tracking, saves CPU |
| Feature detection | Shi-Tomasi | Lightweight, reliable |
| Tracking algorithm | Lucas-Kanade pyramidal (3 levels) | Handles larger motions between frames |
| Re-detection threshold | < 30 good tracked points | Prevents tracking degradation |
| Obstacle ROI | Bottom 45% of frame | Floor-level only, saves CPU |
| Camera role in safety | Advisory only, no hard stops | Prevents false-positive stops |
| Thread architecture | Independent daemon thread | Fully decoupled from SLAM and safety |
| Conflict resolution | Strict priority hierarchy | Safety > camera > SLAM > navigation |

---

*Notes from design discussion — AI Cargo Robot project*
