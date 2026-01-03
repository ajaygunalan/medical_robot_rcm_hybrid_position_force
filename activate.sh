#!/bin/bash
# Activate rcm_force_drake_env without ROS2 pollution
unset PYTHONPATH
source "$(dirname "${BASH_SOURCE[0]}")/rcm_force_drake_env/bin/activate"
