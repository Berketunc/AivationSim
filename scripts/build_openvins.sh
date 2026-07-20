#!/usr/bin/env bash
# Clone (if missing) and build OpenVINS at ~/open_vins, as a sibling colcon
# workspace — same pattern as ~/PX4-Autopilot: kept outside this repo since
# it's a large third-party project, not something we're authoring.
#
# One-time system deps this doesn't install for you (needs sudo):
#   sudo apt install -y libceres-dev libgoogle-glog-dev libatlas-base-dev libsuitesparse-dev
# Everything else it needs (libopencv-dev, libopencv-contrib-dev, libeigen3-dev,
# libboost-dev, ros-jazzy-cv-bridge, ros-jazzy-image-transport,
# ros-jazzy-tf2-geometry-msgs) was already present when this was written.
#
# Usage:  bash scripts/build_openvins.sh

set -euo pipefail

OV_DIR="$HOME/open_vins"

if [[ ! -d "$OV_DIR" ]]; then
    echo "[build_openvins] cloning to $OV_DIR"
    git clone --depth 1 https://github.com/rpng/open_vins.git "$OV_DIR"
else
    echo "[build_openvins] $OV_DIR already exists, skipping clone"
fi

if ! dpkg -s libceres-dev > /dev/null 2>&1; then
    echo "ERROR: libceres-dev not installed. Run:"
    echo "  sudo apt install -y libceres-dev libgoogle-glog-dev libatlas-base-dev libsuitesparse-dev"
    exit 1
fi

set +u
source /opt/ros/jazzy/setup.bash
set -u

cd "$OV_DIR"
echo "[build_openvins] building ov_core, ov_init, ov_msckf (ov_eval/ov_data skipped — not needed for the subscribe-mode node this project uses)"
colcon build --symlink-install --packages-select ov_core ov_init ov_msckf

echo "[build_openvins] done. Source it as an underlay before oa_bringup's overlay:"
echo "  source $OV_DIR/install/setup.bash"
echo "  source /home/berke/AviationSim/precision_landing_ws/install/setup.bash"
