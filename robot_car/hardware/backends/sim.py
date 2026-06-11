"""Simulator backend -- maps the HAL onto the 2D :class:`World`.

Always resolves the current world via ``simulator.get_world()`` so tests which swap
the world with ``reset_world()`` are always reflected.

The camera is the world's raycast first-person feed, rendered from the true pose so
visual odometry recovers real motion. A physical webcam must never be used here: its
view is uncorrelated with the simulated robot, so VO would contradict the encoders
and corrupt the localization fusion (false slip detection, random pose deltas).
"""

from robot_car.core import simulator


class SimBackend:
    name = "sim"

    def start(self) -> None:
        simulator.get_world().start()

    def motor_set(self, left: float, right: float) -> None:
        simulator.get_world().set_motor(left, right)

    def motor_stop(self) -> None:
        simulator.get_world().set_motor(0.0, 0.0)

    def ping_sensor(self, sensor: str) -> float:
        """One sensor 'ping': sample the world's raycast at the current pose (cm).

        The scheduler calls this at the same spaced cadence the real bus runs at, so
        the simulator exercises the real sensing rate rather than an unobtainable
        instantaneous five-sensor sweep.
        """
        return simulator.get_world().read_distance_cm(sensor)

    def read_encoder_pulses(self):
        return simulator.get_world().read_encoder_pulses()

    def camera_read(self):
        return simulator.get_world().render_camera()

    def cleanup(self) -> None:
        world = simulator.get_world()
        world.set_motor(0.0, 0.0)
        world.stop()
