#!/bin/bash
# Launch MuJoCo competition simulation with ROS2 bridge
# Usage: ./launch_competition_sim.sh [--headless] [--images]

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

# Source ROS2
source /opt/ros/humble/setup.bash

# Parse arguments
HEADLESS=false
IMAGES=""
while [[ $# -gt 0 ]]; do
    case $1 in
        --headless)
            HEADLESS=true
            shift
            ;;
        --images)
            IMAGES="--ros2-images"
            shift
            ;;
        --sitl)
            SITL_ARG="--sitl"
            shift
            ;;
        *)
            echo "Unknown argument: $1"
            # Pass unknown args to python script directly?
            # Or just ignore? Let's assume user might pass other py args.
            # But the loop consumes $1.
            # Let's just collect unknown args.
            EXTRA_ARGS="$EXTRA_ARGS $1"
            shift
            ;;
    esac
done

# Set display for headless mode
if [ "$HEADLESS" = true ]; then
    export DISPLAY=""
    export MUJOCO_GL=egl
    echo "[launch] Running in headless mode (EGL rendering)"
fi

echo "[launch] Starting MuJoCo UUV Competition Simulation"
echo "[launch] Scene: competition_scene.xml"
echo "[launch] ROS2 Topics:"
echo "  Input:  /cmd_vel, /mavros/rc/override"
echo "  Output: /imu/data, /dvl/velocity, /dvl/odometry, /dvl/altitude"
echo "  Debug:  /mujoco/ground_truth/pose"
if [ -n "$IMAGES" ]; then
    echo "  Images: /stereo/left/image_raw, /stereo/right/image_raw"
fi
echo ""

python3 run_urdf_full.py \
    --scene competition_scene.xml \
    --ros2 $IMAGES $SITL_ARG \
    --ros2-sensor-hz 50 \
    --ros2-image-hz 15 \
    "$@"
