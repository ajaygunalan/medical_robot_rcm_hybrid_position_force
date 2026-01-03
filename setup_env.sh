#!/bin/bash
set -e
cd "$(dirname "${BASH_SOURCE[0]}")"

# Clear ROS2 pollution
unset PYTHONPATH

python3 -m venv rcm_force_drake_env
source rcm_force_drake_env/bin/activate
pip install --upgrade pip
pip install drake manipulation ur-rtde --extra-index-url https://drake-packages.csail.mit.edu/whl/nightly/

echo "Done. Run: source rcm_force_drake_env/bin/activate"
