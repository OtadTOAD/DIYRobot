"""USB camera: capture + visual odometry + appearance detection + debug view.

One daemon thread captures frames and feeds two independent pipelines that share the
same frame stream (camera_integration_design.md):

  Pipeline 1 -- Visual odometry (F-07): Shi-Tomasi corners + Lucas-Kanade pyramidal
    optical flow -> (dx, dy, dtheta) robot-frame delta + confidence -> state.vo_*.
  Pipeline 2 -- Appearance detection (F-08): ROI Canny + contour analysis -> a soft
    advisory flag for the safety monitor (state.camera_advisory). Advisory only --
    never a hard stop.

The pure functions :func:`estimate_motion` and :func:`detect_appearance` are unit
tested directly. The thread also publishes an annotated ``debug_frame`` (tracked
features, contours, VO text) consumed by the web ``/camera/debug.mjpg`` stream so the
camera's behaviour is observable headless; with ``SHOW_CAMERA_WINDOW=1`` it also opens
a live ``cv2.imshow`` window.
"""

from __future__ import annotations

import math
import threading
import time

import cv2
import numpy as np

from robot_car import config, state
from robot_car.hardware import hal


# ---------------------------------------------------------------------------
# Pipeline 1 -- Visual odometry (pure, testable)
# ---------------------------------------------------------------------------
def detect_features(gray: np.ndarray) -> np.ndarray | None:
    return cv2.goodFeaturesToTrack(
        gray,
        maxCorners=config.VO_MAX_CORNERS,
        qualityLevel=config.VO_QUALITY_LEVEL,
        minDistance=config.VO_MIN_DISTANCE,
        blockSize=config.VO_BLOCK_SIZE,
    )


def estimate_motion(prev_gray, curr_gray, prev_features):
    """Estimate the world-frame motion delta (dx, dy, dtheta) between two frames.

    Tracks features with Lucas-Kanade, fits a partial-affine (rotation + translation)
    transform mapping the previous points to the current ones, then converts the
    image motion of the frame centre into a world-frame robot displacement using the
    camera scale. Returns ``(estimate, confidence, good_prev, good_next)``.
    """
    zero = ((0.0, 0.0, 0.0), 0.0, None, None)
    if prev_features is None or len(prev_features) == 0:
        return zero

    next_pts, status, _ = cv2.calcOpticalFlowPyrLK(
        prev_gray, curr_gray, prev_features, None,
        winSize=config.VO_LK_WIN, maxLevel=config.VO_LK_MAX_LEVEL,
        criteria=(cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 30, 0.01),
    )
    if next_pts is None or status is None:
        return zero

    s = status.flatten() == 1
    good_prev = prev_features[s]
    good_next = next_pts[s]
    if len(good_prev) < config.VO_MIN_TRACKED:
        return (0.0, 0.0, 0.0), 0.0, good_prev, good_next

    p0 = good_prev.reshape(-1, 2).astype(np.float32)
    p1 = good_next.reshape(-1, 2).astype(np.float32)
    matrix, inliers = cv2.estimateAffinePartial2D(p0, p1, method=cv2.RANSAC)
    if matrix is None:
        return (0.0, 0.0, 0.0), 0.0, good_prev, good_next

    # Image rotation between frames maps directly to robot yaw under our render
    # convention (and the forward camera after calibration on the real Pi).
    dtheta = math.atan2(matrix[1, 0], matrix[0, 0])

    # Displacement of the frame centre under the affine transform -> world motion.
    h, w = prev_gray.shape[:2]
    centre = np.array([w / 2.0, h / 2.0, 1.0])
    disp_x = float(matrix[0] @ centre) - w / 2.0
    disp_y = float(matrix[1] @ centre) - h / 2.0
    scale = config.VO_PIXELS_PER_METRE
    dx = -disp_x / scale
    dy = disp_y / scale

    conf = motion_confidence(len(good_prev), inliers)
    return (dx, dy, dtheta), conf, good_prev, good_next


def motion_confidence(n_tracked: int, inliers) -> float:
    """Confidence in [0, 1] from tracked feature count and RANSAC inlier ratio.

    The inlier ratio captures how well a single rigid motion explains the flow --
    high for clean translation or rotation, low for inconsistent (shaky / textureless)
    motion. Robust to axis-aligned flow, unlike a raw flow-std measure.
    """
    if n_tracked < config.VO_MIN_TRACKED or inliers is None:
        return 0.0
    inlier_ratio = float(np.mean(inliers))
    coverage = min(1.0, n_tracked / config.VO_MAX_CORNERS)
    return float(np.clip(coverage * inlier_ratio, 0.0, 1.0))


# ---------------------------------------------------------------------------
# Pipeline 2 -- Appearance-based obstacle detection (pure, testable)
# ---------------------------------------------------------------------------
def detect_appearance(frame: np.ndarray):
    """Return ``(advisory: bool, contours)`` from the lower ROI of the frame."""
    h, w = frame.shape[:2]
    roi = frame[int(h * config.APPEARANCE_ROI_TOP):h, :]
    gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY) if roi.ndim == 3 else roi
    blurred = cv2.GaussianBlur(gray, config.APPEARANCE_BLUR, 0)
    edges = cv2.Canny(blurred, config.APPEARANCE_CANNY_LOW, config.APPEARANCE_CANNY_HIGH)
    # Close small gaps so an object's silhouette forms a single enclosing contour
    # rather than fragmenting into many tiny edge pieces.
    edges = cv2.dilate(edges, np.ones((3, 3), np.uint8), iterations=2)
    contours, _ = cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    roi_w = roi.shape[1]
    advisory = False
    significant = []
    for c in contours:
        if cv2.contourArea(c) < config.APPEARANCE_MIN_CONTOUR_AREA:
            continue
        significant.append(c)
        x, y, cw, ch = cv2.boundingRect(c)
        cx = x + cw / 2.0
        if roi_w * 0.25 < cx < roi_w * 0.75:   # lower-centre region
            advisory = True
    return advisory, significant


# ---------------------------------------------------------------------------
# Camera thread
# ---------------------------------------------------------------------------
class CameraThread(threading.Thread):
    """Daemon thread: capture frames, run both pipelines, publish debug frame."""

    def __init__(self):
        super().__init__(name="camera", daemon=True)
        self._prev_gray = None
        self._features = None
        self._roi_offset = 0

    def run(self) -> None:
        backend = hal.get_backend()
        period = 1.0 / config.CAMERA_HZ
        while not state.stop_event.is_set():
            t0 = time.monotonic()
            frame = backend.camera_read()
            if frame is None:
                time.sleep(period)
                continue
            state.set_latest_frame(frame)
            self._process(frame)
            elapsed = time.monotonic() - t0
            if elapsed < period:
                time.sleep(period - elapsed)
        if config.SHOW_CAMERA_WINDOW:
            cv2.destroyAllWindows()

    def _process(self, frame) -> None:
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        # --- Pipeline 1: visual odometry ---
        good_prev = good_next = None
        if self._prev_gray is not None and self._features is not None:
            est, conf, good_prev, good_next = estimate_motion(
                self._prev_gray, gray, self._features
            )
            state.set_vo(est, conf)
            # Re-detect when we are running low on tracked points.
            if good_next is None or len(good_next) < config.VO_REDETECT_THRESHOLD:
                self._features = detect_features(gray)
            else:
                self._features = good_next.reshape(-1, 1, 2)
        else:
            self._features = detect_features(gray)
        self._prev_gray = gray

        # --- Pipeline 2: appearance detection ---
        advisory, contours = detect_appearance(frame)
        state.set_advisory(advisory)

        # --- Debug overlay ---
        self._publish_debug(frame, good_next, contours, advisory)

    def _publish_debug(self, frame, good_next, contours, advisory) -> None:
        dbg = frame.copy()
        if good_next is not None:
            for pt in good_next.reshape(-1, 2):
                cv2.circle(dbg, (int(pt[0]), int(pt[1])), 2, (0, 255, 0), -1)
        h = dbg.shape[0]
        roi_top = int(h * config.APPEARANCE_ROI_TOP)
        cv2.line(dbg, (0, roi_top), (dbg.shape[1], roi_top), (255, 200, 0), 1)
        if contours:
            shifted = [c + np.array([[0, roi_top]]) for c in contours]
            cv2.drawContours(dbg, shifted, -1, (0, 0, 255), 2)
        est, conf = state.get_vo()
        cv2.putText(dbg, "VO dx=%.3f dth=%.3f conf=%.2f" % (est[0], est[2], conf),
                    (8, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)
        if advisory:
            cv2.putText(dbg, "ADVISORY", (8, 44),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
        state.set_debug_frame(dbg)
        if config.SHOW_CAMERA_WINDOW:
            cv2.imshow("camera debug", dbg)
            cv2.waitKey(1)


def start_camera() -> CameraThread:
    thread = CameraThread()
    thread.start()
    return thread
