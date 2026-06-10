"""Raspberry Pi backend -- real GPIO via pigpio.

Only imported when a Pi is detected (see hal.py), so ``import pigpio`` here is safe.
Uses pigpio because it gives stable hardware PWM on GPIO 12/13 and precise edge
callbacks for the ultrasonic echo timing and the encoder pulse counting -- both of
which jitter badly under RPi.GPIO when the CPU is busy with SLAM and the web server.

Requires the pigpio daemon to be running:  ``sudo pigpiod``
"""

import threading
import time

import pigpio

from robot_car import config


class RealBackend:
    name = "pi"

    def __init__(self):
        self.pi = pigpio.pi()
        if not self.pi.connected:
            raise RuntimeError(
                "pigpio daemon not reachable -- run 'sudo pigpiod' before starting."
            )
        self._enc_left = 0
        self._enc_right = 0
        self._enc_lock = threading.Lock()
        self._cam = None
        self._setup_gpio()

    # -- setup ---------------------------------------------------------------
    def _setup_gpio(self) -> None:
        pi = self.pi
        for pin in (config.PIN_IN1, config.PIN_IN2, config.PIN_IN3, config.PIN_IN4):
            pi.set_mode(pin, pigpio.OUTPUT)
            pi.write(pin, 0)
        pi.set_mode(config.PIN_ENA, pigpio.OUTPUT)
        pi.set_mode(config.PIN_ENB, pigpio.OUTPUT)

        # Ultrasonic TRIG out, ECHO in.
        for trig, echo in config.SENSOR_PINS.values():
            pi.set_mode(trig, pigpio.OUTPUT)
            pi.write(trig, 0)
            pi.set_mode(echo, pigpio.INPUT)

        # Encoders: count rising edges via callbacks.
        pi.set_mode(config.PIN_ENCODER_LEFT, pigpio.INPUT)
        pi.set_mode(config.PIN_ENCODER_RIGHT, pigpio.INPUT)
        pi.set_pull_up_down(config.PIN_ENCODER_LEFT, pigpio.PUD_DOWN)
        pi.set_pull_up_down(config.PIN_ENCODER_RIGHT, pigpio.PUD_DOWN)
        self._cb_left = pi.callback(config.PIN_ENCODER_LEFT, pigpio.RISING_EDGE,
                                    self._on_left_pulse)
        self._cb_right = pi.callback(config.PIN_ENCODER_RIGHT, pigpio.RISING_EDGE,
                                     self._on_right_pulse)

    def start(self) -> None:
        pass  # GPIO already initialised in __init__

    # -- motors --------------------------------------------------------------
    def _set_channel(self, in_a, in_b, en, value) -> None:
        """value in -1..1; sign sets direction, magnitude sets PWM duty."""
        value = max(-1.0, min(1.0, value))
        forward = value >= 0
        self.pi.write(in_a, 1 if forward else 0)
        self.pi.write(in_b, 0 if forward else 1)
        duty = int(abs(value) * 1_000_000)          # pigpio hardware PWM: 0..1e6
        self.pi.hardware_PWM(en, config.PWM_FREQUENCY, duty)

    def motor_set(self, left: float, right: float) -> None:
        self._set_channel(config.PIN_IN1, config.PIN_IN2, config.PIN_ENA, left)
        self._set_channel(config.PIN_IN3, config.PIN_IN4, config.PIN_ENB, right)

    def motor_stop(self) -> None:
        self.pi.hardware_PWM(config.PIN_ENA, config.PWM_FREQUENCY, 0)
        self.pi.hardware_PWM(config.PIN_ENB, config.PWM_FREQUENCY, 0)
        for pin in (config.PIN_IN1, config.PIN_IN2, config.PIN_IN3, config.PIN_IN4):
            self.pi.write(pin, 0)

    # -- ultrasonic ----------------------------------------------------------
    def read_distance_cm(self, sensor: str) -> float:
        trig, echo = config.SENSOR_PINS[sensor]
        # 10 us trigger pulse.
        self.pi.gpio_trigger(trig, 10, 1)
        timeout = config.SENSOR_TIMEOUT_MS / 1000.0

        start = time.time()
        while self.pi.read(echo) == 0:
            if time.time() - start > timeout:
                return float("inf")
        echo_start = time.time()
        while self.pi.read(echo) == 1:
            if time.time() - echo_start > timeout:
                return float("inf")
        echo_end = time.time()

        dist_cm = (echo_end - echo_start) * config.SPEED_OF_SOUND_CM_S / 2.0
        if dist_cm > config.SENSOR_MAX_RANGE_CM or dist_cm <= 0:
            return float("inf")
        return dist_cm

    # -- encoders ------------------------------------------------------------
    def _on_left_pulse(self, gpio, level, tick) -> None:
        with self._enc_lock:
            self._enc_left += 1

    def _on_right_pulse(self, gpio, level, tick) -> None:
        with self._enc_lock:
            self._enc_right += 1

    def read_encoder_pulses(self):
        with self._enc_lock:
            l, r = self._enc_left, self._enc_right
            self._enc_left = 0
            self._enc_right = 0
            return l, r

    # -- camera --------------------------------------------------------------
    def camera_read(self):
        import cv2
        if self._cam is None:
            self._cam = cv2.VideoCapture(config.CAMERA_INDEX)
            self._cam.set(cv2.CAP_PROP_FRAME_WIDTH, config.CAMERA_WIDTH)
            self._cam.set(cv2.CAP_PROP_FRAME_HEIGHT, config.CAMERA_HEIGHT)
            self._cam.set(cv2.CAP_PROP_FPS, config.CAMERA_FPS)
            if not self._cam.isOpened():
                raise RuntimeError(
                    "USB camera not found at /dev/video%d -- check the connection."
                    % config.CAMERA_INDEX
                )
        ok, frame = self._cam.read()
        return frame if ok else None

    # -- lifecycle -----------------------------------------------------------
    def cleanup(self) -> None:
        try:
            self.motor_stop()
        finally:
            for cb in (getattr(self, "_cb_left", None), getattr(self, "_cb_right", None)):
                if cb is not None:
                    cb.cancel()
            if self._cam is not None:
                self._cam.release()
            self.pi.stop()
