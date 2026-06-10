"""Simulator backend -- maps the HAL onto the 2D :class:`World`.

Motors / sensors / encoders always resolve the current world via
``simulator.get_world()`` so that tests which swap the world with ``reset_world()``
are always reflected.

The camera is the one exception: rather than always synthesising a noise feed, the
backend first probes for a *real* USB camera (``config.SIM_PREFER_REAL_CAMERA``) and
streams it when present -- so running on a laptop with a webcam shows the actual
camera, and the synthetic feed is used only as a fallback when no camera is found.
"""

import contextlib

from robot_car import config, state
from robot_car.core import simulator


@contextlib.contextmanager
def _silenced_opencv_logs(cv2):
    """Mute OpenCV's stderr chatter while probing for a camera that may not exist.

    ``cv2.VideoCapture`` never raises when a device is absent -- it logs V4L2/FFMPEG
    warnings to stderr and returns a closed handle. During a *probe* those warnings
    are expected and noisy, so we silence OpenCV's logger for the duration.
    """
    setter = getattr(getattr(cv2, "utils", None), "logging", None)
    prev = None
    try:
        if setter is not None:
            prev = setter.getLogLevel()
            setter.setLogLevel(setter.LOG_LEVEL_SILENT)
        elif hasattr(cv2, "setLogLevel"):
            cv2.setLogLevel(0)            # 0 == LOG_LEVEL_SILENT
    except Exception:
        prev = None
    try:
        yield
    finally:
        if prev is not None:
            try:
                setter.setLogLevel(prev)
            except Exception:
                pass


class SimBackend:
    name = "sim"

    def __init__(self):
        self._cam = None            # cv2.VideoCapture once a real camera is found
        self._cam_probed = False    # have we attempted to open a real camera yet?

    def start(self) -> None:
        # Start the real-time physics thread (no-op if already running).
        simulator.get_world().start()

    def motor_set(self, left: float, right: float) -> None:
        simulator.get_world().set_motor(left, right)

    def motor_stop(self) -> None:
        simulator.get_world().set_motor(0.0, 0.0)

    def read_distance_cm(self, sensor: str) -> float:
        return simulator.get_world().read_distance_cm(sensor)

    def read_encoder_pulses(self):
        return simulator.get_world().read_encoder_pulses()

    def camera_read(self):
        cam = self._real_camera()
        if cam is not None:
            ok, frame = cam.read()
            if ok and frame is not None:
                return frame
            # Real camera dropped out mid-run -- fall back to the synthetic feed.
        return simulator.get_world().render_camera()

    def _real_camera(self):
        """Lazily open a real USB camera once; cache the decision.

        Returns the live ``cv2.VideoCapture`` if a camera is attached, else ``None``
        (meaning: use the synthetic feed). Probed only once so a missing camera
        doesn't cost a device-open attempt every frame.
        """
        if self._cam_probed:
            return self._cam
        self._cam_probed = True
        if not config.SIM_PREFER_REAL_CAMERA:
            return None
        try:
            import cv2
            # Prefer the V4L2 backend on Linux so a missing device fails fast instead
            # of falling through to FFMPEG (which logs an "index out of range" error).
            api = getattr(cv2, "CAP_V4L2", cv2.CAP_ANY)
            with _silenced_opencv_logs(cv2):
                cap = cv2.VideoCapture(config.CAMERA_INDEX, api)
                cap.set(cv2.CAP_PROP_FRAME_WIDTH, config.CAMERA_WIDTH)
                cap.set(cv2.CAP_PROP_FRAME_HEIGHT, config.CAMERA_HEIGHT)
                cap.set(cv2.CAP_PROP_FPS, config.CAMERA_FPS)
                ok, frame = (cap.read() if cap.isOpened() else (False, None))
            if ok and frame is not None:
                self._cam = cap
                state.set_log("ok", "Using real USB camera at /dev/video%d"
                              % config.CAMERA_INDEX)
            else:
                cap.release()
                state.set_log("info", "No USB camera found -- using synthetic feed")
        except Exception as exc:                          # pragma: no cover
            state.set_log("info", "Camera probe failed (%s) -- synthetic feed" % exc)
            self._cam = None
        return self._cam

    def cleanup(self) -> None:
        world = simulator.get_world()
        world.set_motor(0.0, 0.0)
        world.stop()
        if self._cam is not None:
            self._cam.release()
            self._cam = None
