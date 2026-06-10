"""Forbidden zones -- a constraint layer kept separate from the map (F-13).

User-drawn no-go rectangles live in their own boolean grid, never baked into the
occupancy grid, so they can be edited or cleared without re-exploring. A* treats a
hard-blocked cell exactly like a wall. A soft-penalty mode is available for tight
spaces where hard blocking could make a goal unreachable (web_interface_design.md).

Saved alongside the map as a ``.zones`` file. Coordinates are grid cells ``(col, row)``.
"""

from __future__ import annotations

import msgpack
import numpy as np

from robot_car import config


class ForbiddenZones:
    def __init__(self, width=None, height=None):
        self.width = int(width or config.MAP_WIDTH_CELLS)
        self.height = int(height or config.MAP_HEIGHT_CELLS)
        self.mask = np.zeros((self.height, self.width), dtype=bool)
        self.hard_block = True          # False => soft penalty (handled by A* cost)

    def add_zone(self, x1: int, y1: int, x2: int, y2: int) -> None:
        c0, c1 = sorted((int(x1), int(x2)))
        r0, r1 = sorted((int(y1), int(y2)))
        c0 = max(0, c0); r0 = max(0, r0)
        c1 = min(self.width - 1, c1); r1 = min(self.height - 1, r1)
        self.mask[r0:r1 + 1, c0:c1 + 1] = True

    def clear_all(self) -> None:
        self.mask[:] = False

    def is_blocked(self, col: int, row: int) -> bool:
        if 0 <= col < self.width and 0 <= row < self.height:
            return bool(self.mask[row, col])
        return False

    def count(self) -> int:
        return int(self.mask.sum())

    # -- persistence ---------------------------------------------------------
    def save(self, path: str) -> None:
        payload = {
            "width": self.width,
            "height": self.height,
            "hard_block": self.hard_block,
            "mask": np.packbits(self.mask).tobytes(),
        }
        with open(path, "wb") as fh:
            fh.write(msgpack.packb(payload, use_bin_type=True))

    def load(self, path: str) -> None:
        with open(path, "rb") as fh:
            data = msgpack.unpackb(fh.read(), raw=False)
        self.width = data["width"]
        self.height = data["height"]
        self.hard_block = data.get("hard_block", True)
        bits = np.frombuffer(data["mask"], dtype=np.uint8)
        flat = np.unpackbits(bits, count=self.width * self.height).astype(bool)
        self.mask = flat.reshape(self.height, self.width)
