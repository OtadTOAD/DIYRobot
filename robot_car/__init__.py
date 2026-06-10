"""AI Cargo Robot -- classical-AI autonomous indoor cargo robot for Raspberry Pi 4B.

Package layout:
    config.py / state.py  -- configuration and locked shared state
    hardware/             -- HAL (motors, sensors, camera) + pi/sim backends
    core/                 -- occupancy grid, SLAM, localization, A*, safety, simulator
    modes/                -- explore / navigate / idle behaviours
    ui/                   -- Flask + SocketIO web interface
"""

__version__ = "1.0.0"
