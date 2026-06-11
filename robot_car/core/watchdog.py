"""Thread supervisor (P0-3) -- turns silent thread death into a loud, safe failure.

All the core loops are daemon threads started once; nothing used to notice if one
died. A dead safety thread is the worst case: the motors keep executing the
navigator's last command and the layer that "overrides everything" is silently gone.

Two cheap mechanisms cover this:
  * the motor layer refuses to drive when the *safety* heartbeat is stale
    (``motors._safety_alive``), so a dead safety thread coasts the robot to a stop;
  * this watchdog polls every thread's heartbeat and reports a stall to the status
    log exactly once (and again when it recovers), so SLAM / camera / sensor death
    is visible in the web UI instead of manifesting as a frozen pose or map.
"""

from __future__ import annotations

import threading

from robot_car import config, state
from robot_car.hardware import motors


class Watchdog(threading.Thread):
    def __init__(self, hz: float = config.WATCHDOG_HZ):
        super().__init__(name="watchdog", daemon=True)
        self.period = 1.0 / hz
        self._stalled: set = set()

    def run(self) -> None:
        while not state.stop_event.is_set():
            self._check()
            state.stop_event.wait(self.period)

    def _check(self) -> None:
        """One supervision pass: report newly stalled threads and recoveries once."""
        for name, age in state.heartbeats().items():
            if age > config.WATCHDOG_STALE_S and name not in self._stalled:
                self._stalled.add(name)
                state.set_log("error",
                              "Thread '%s' stalled (%.1fs) -- check the robot"
                              % (name, age))
                if name == "safety":
                    motors.stop()               # safety gone: make sure we are stopped
            elif age <= config.WATCHDOG_STALE_S and name in self._stalled:
                self._stalled.discard(name)
                state.set_log("ok", "Thread '%s' recovered" % name)


def start_watchdog() -> Watchdog:
    watchdog = Watchdog()
    watchdog.start()
    return watchdog
