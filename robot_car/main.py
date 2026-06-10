"""Entry point and CLI (F-20).

Usage (simulator on a laptop, or real hardware on a Pi -- same command):

    python -m robot_car.main                      # idle, open the web UI
    python -m robot_car.main --explore            # start autonomous exploration
    python -m robot_car.main --map living_room    # load a saved map, then idle
    python -m robot_car.main --map living_room --waypoint desk   # load + navigate

Force a backend with the environment variable ROBOT_BACKEND=pi|sim (default: auto).
"""

import argparse
import os
import sys

# Allow `python robot_car/main.py` as well as `python -m robot_car.main`.
if __package__ in (None, ""):
    sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))


def parse_args(argv=None):
    p = argparse.ArgumentParser(description="AI Cargo Robot")
    p.add_argument("--explore", action="store_true", help="start in exploration mode")
    p.add_argument("--map", dest="map_name", default=None, help="load a saved map by name")
    p.add_argument("--waypoint", default=None, help="navigate to a named waypoint on start")
    return p.parse_args(argv)


def main(argv=None) -> None:
    args = parse_args(argv)
    from robot_car.app import run
    run(explore=args.explore, map_name=args.map_name, waypoint=args.waypoint)


if __name__ == "__main__":
    main()
