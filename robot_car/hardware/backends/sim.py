"""Simulator backend -- maps the HAL onto the 2D :class:`World`.

Stateless beyond a reference to the simulator singleton; every call resolves the
current world via ``simulator.get_world()`` so that tests which swap the world with
``reset_world()`` are always reflected.
"""

from robot_car.core import simulator


class SimBackend:
    name = "sim"

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
        return simulator.get_world().render_camera()

    def cleanup(self) -> None:
        world = simulator.get_world()
        world.set_motor(0.0, 0.0)
        world.stop()
