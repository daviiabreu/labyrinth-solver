#!/bin/bash
cd "$(dirname "$0")"
source install/setup.bash
export DYLD_LIBRARY_PATH="/opt/micromamba/envs/ros_env/lib:/opt/micromamba/envs/ros_env/lib/python3.11/site-packages:$(pwd)/install/lib:$DYLD_LIBRARY_PATH"
export PYTHONHOME="/opt/micromamba/envs/ros_env"
export PYTHONPATH="/opt/micromamba/envs/ros_env/lib/python3.11/site-packages:$PYTHONPATH"
export LD_LIBRARY_PATH="/opt/micromamba/envs/ros_env/lib:$(pwd)/install/lib:$LD_LIBRARY_PATH"
export DYLD_INSERT_LIBRARIES="/opt/micromamba/envs/ros_env/lib/libpython3.11.dylib"
exec install/cg_solver/lib/cg_solver/part2_mapper "$@"
