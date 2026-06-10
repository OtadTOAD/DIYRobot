"""Encoder dead-reckoning odometry (F-04).

Differential-drive kinematics from wheel pulse counts. The LM393 encoders are
single-channel and cannot report direction; pulses arrive already signed by the
motor command active when they were produced (see hardware/motors).

An :class:`Odometry` instance maintains the encoder-only pose estimate. The SLAM /
localization thread owns one instance, calls :meth:`update` once per cycle, and
feeds the result into the three-source fusion (F-11). On its own this is the "base
motion estimate" of the localization stack.
"""

import math

from robot_car import config
from robot_car.core.geometry import wrap_angle
from robot_car.hardware import motors


class Odometry:
    def __init__(self, start_pose=(0.0, 0.0, 0.0)):
        self.x, self.y, self.theta = (float(v) for v in start_pose)
        self.last_distance = 0.0   # |centre displacement| last update (slip checks)

    def set_pose(self, pose) -> None:
        self.x, self.y, self.theta = (float(v) for v in pose)

    def get_pose(self) -> tuple:
        return (self.x, self.y, self.theta)

    def update(self) -> tuple:
        """Consume encoder pulses and advance the pose. Returns the new pose."""
        left_pulses, right_pulses = motors.consume_signed_pulses()
        delta_left = left_pulses * config.DIST_PER_PULSE
        delta_right = right_pulses * config.DIST_PER_PULSE

        delta_c = (delta_left + delta_right) / 2.0
        delta_theta = (delta_right - delta_left) / config.WHEEL_BASE

        # Integrate around the mid-heading for a better arc approximation.
        mid = self.theta + delta_theta / 2.0
        self.x += delta_c * math.cos(mid)
        self.y += delta_c * math.sin(mid)
        self.theta = wrap_angle(self.theta + delta_theta)
        self.last_distance = abs(delta_c)
        return (self.x, self.y, self.theta)
