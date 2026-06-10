#!/bin/bash
set -e

cd "$(dirname "$0")"

if [ ! -d ".venv" ]; then
    echo "Creating virtual environment..."
    python -m venv .venv
    . .venv/bin/activate
    pip install -r robot_car/requirements.txt
else
    . .venv/bin/activate
fi

python -m robot_car.main "$@"
