"""Raspberry Pi backend -- real GPIO via pigpio.

Only imported when a Pi is detected (see hal.py), so ``import pigpio`` here is safe.
Uses pigpio because it gives stable hardware PWM on GPIO 12/13 and precise edge
callbacks for the ultrasonic echo timing and the encoder pulse counting -- both of
which jitter badly under RPi.GPIO when the CPU is busy with SLAM and the web server.

Requires the pigpio daemon to be running:  ``sudo pigpiod``
"""

import threading

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
        # Per-sensor echo-edge state, populated by persistent pigpio callbacks.
        self._echo_rise = {}            # sensor -> tick of the last rising edge
        self._echo_width = {}           # sensor -> last measured pulse width (us)
        self._echo_done = {}            # sensor -> Event set when a pulse completes
        self._echo_cbs = []
        self._setup_gpio()

    # -- setup ---------------------------------------------------------------
    def _setup_gpio(self) -> None:
        pi = self.pi
        for pin in (config.PIN_IN1, config.PIN_IN2, config.PIN_IN3, config.PIN_IN4):
            pi.set_mode(pin, pigpio.OUTPUT)
            pi.write(pin, 0)
        pi.set_mode(config.PIN_ENA, pigpio.OUTPUT)
        pi.set_mode(config.PIN_ENB, pigpio.OUTPUT)

        # Ultrasonic TRIG out, ECHO in. Echo timing is done with persistent
        # EITHER_EDGE callbacks: the daemon timestamps both edges in hardware and
        # delivers microsecond ticks, so ``ping_sensor`` just triggers and waits for
        # the pulse to complete -- no socket polling, no time.time() jitter (P0-1c).
        for name, (trig, echo) in config.SENSOR_PINS.items():
            pi.set_mode(trig, pigpio.OUTPUT)
            pi.write(trig, 0)
            pi.set_mode(echo, pigpio.INPUT)
            self._echo_done[name] = threading.Event()
            self._echo_cbs.append(
                pi.callback(echo, pigpio.EITHER_EDGE, self._make_echo_cb(name))
            )

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
    def _make_echo_cb(self, sensor: str):
        """Build the EITHER_EDGE callback that times one sensor's echo pulse."""
        def _cb(gpio, level, tick):
            if level == 1:                       # rising edge: echo started
                self._echo_rise[sensor] = tick
            elif level == 0:                     # falling edge: echo finished
                rise = self._echo_rise.get(sensor)
                if rise is not None:
                    self._echo_width[sensor] = pigpio.tickDiff(rise, tick)
                    self._echo_done[sensor].set()
        return _cb

    def ping_sensor(self, sensor: str) -> float:
        """Trigger one HC-SR04 and return the echo distance in cm (``inf`` on timeout).

        The scheduler guarantees only one ping is in flight at a time, so the shared
        callback state is never raced across sensors.
        """
        trig, _ = config.SENSOR_PINS[sensor]
        done = self._echo_done[sensor]
        done.clear()
        self._echo_rise.pop(sensor, None)
        self.pi.gpio_trigger(trig, 10, 1)        # 10 us trigger pulse
        if not done.wait(config.SENSOR_TIMEOUT_MS / 1000.0):
            return float("inf")
        width_us = self._echo_width.get(sensor)
        if width_us is None:
            return float("inf")
        dist_cm = (width_us / 1_000_000.0) * config.SPEED_OF_SOUND_CM_S / 2.0
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
            for cb in self._echo_cbs:
                cb.cancel()
            if self._cam is not None:
                self._cam.release()
            self.pi.stop()
