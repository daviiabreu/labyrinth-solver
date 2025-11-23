#!/bin/bash
cd "$(dirname "$0")"
INSTALL_DIR="install/cg_solver/lib/cg_solver"
LIB_DIR="install/lib"
ABS_INSTALL_DIR="$(pwd)/install/lib"
for lib in "$LIB_DIR"/*cg_interfaces*.dylib "$LIB_DIR"/*rcl_interfaces*.dylib "$LIB_DIR"/*rosgraph_msgs*.dylib "$LIB_DIR"/*statistics_msgs*.dylib "$LIB_DIR"/*builtin_interfaces*.dylib; do
    if [ -f "$lib" ]; then
        install_name_tool -add_rpath "$ABS_INSTALL_DIR" "$lib" 2>/dev/null || true
        install_name_tool -add_rpath "/opt/micromamba/envs/ros_env/lib" "$lib" 2>/dev/null || true
    fi
done
for exe in "$INSTALL_DIR"/part*; do
    if [ -f "$exe" ]; then
        install_name_tool -add_rpath "$ABS_INSTALL_DIR" "$exe" 2>/dev/null || true
        install_name_tool -add_rpath "/opt/micromamba/envs/ros_env/lib" "$exe" 2>/dev/null || true
    fi
done
