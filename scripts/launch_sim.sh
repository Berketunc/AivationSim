#!/usr/bin/env bash
# Launch PX4 SITL + Gazebo Harmonic, then the ROS 2 precision landing stack.
# Ctrl-C tears everything down cleanly.
#
# Usage:  bash scripts/launch_sim.sh

set -uo pipefail

REPO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
PX4_DIR="$HOME/PX4-Autopilot"
WS_DIR="$REPO_DIR/precision_landing_ws"
PX4_LOG="/tmp/px4_sitl.log"

# ── pre-flight checks ──────────────────────────────────────────────────────────
if [[ ! -f "$WS_DIR/install/setup.bash" ]]; then
    echo "ERROR: workspace not built. Run:"
    echo "  cd $WS_DIR && colcon build --symlink-install"
    exit 1
fi

if [[ ! -d "$PX4_DIR" ]]; then
    echo "ERROR: PX4-Autopilot not found at $PX4_DIR"
    exit 1
fi

# ── cleanup on exit ────────────────────────────────────────────────────────────
cleanup() {
    echo -e "\n[sim] shutting down..."
    pkill -f "px4_sitl_default/bin/px4" 2>/dev/null || true
    pkill -f "gz sim"                    2>/dev/null || true
    pkill -f "gz-sim-server"             2>/dev/null || true
    pkill -f "gz-sim-gui"                2>/dev/null || true
    wait 2>/dev/null
    echo "[sim] done."
}
trap cleanup EXIT INT TERM

# ── 1. PX4 + Gazebo ───────────────────────────────────────────────────────────
echo "[1/2] starting PX4 SITL + Gazebo Harmonic (world: aruco)…"
> "$PX4_LOG"
( cd "$PX4_DIR" && PX4_GZ_WORLD=aruco PX4_GZ_MODEL_POSE="5,0,0,0,0,0" make px4_sitl gz_x500_mono_cam_down ) \
    2>&1 | tee "$PX4_LOG" &

echo "[1/2] waiting for PX4 MAVLink to come up (up to 60 s)…"
WAITED=0
until grep -q "mode: Onboard" "$PX4_LOG" 2>/dev/null; do
    sleep 1
    WAITED=$((WAITED + 1))
    if [[ $WAITED -ge 60 ]]; then
        echo "ERROR: PX4 didn't start within 60 s. Check $PX4_LOG for details."
        exit 1
    fi
done
echo "[1/2] PX4 MAVLink up — waiting 10 s for EKF2 to converge…"
sleep 10

# ── 2. ROS 2 precision landing stack ──────────────────────────────────────────
echo "[2/2] starting ROS 2 precision landing stack…"
set +u  # ROS setup scripts reference variables that may not be set yet
source /opt/ros/jazzy/setup.bash
source "$WS_DIR/install/setup.bash"
set -u
ros2 launch pl_bringup sim_precision_landing.launch.py
