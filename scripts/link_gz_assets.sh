#!/usr/bin/env bash
# Symlink this repo's Gazebo worlds/models (sim_assets/) into the PX4-Autopilot
# clone so `make px4_sitl gz_<model>` can find them. Assets live in this repo
# so they're version-controlled; PX4-Autopilot itself is gitignored here.
#
# Usage:  bash scripts/link_gz_assets.sh

set -uo pipefail

REPO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
PX4_DIR="$HOME/PX4-Autopilot"
ASSETS_DIR="$REPO_DIR/sim_assets"

if [[ ! -d "$PX4_DIR" ]]; then
    echo "ERROR: PX4-Autopilot not found at $PX4_DIR"
    exit 1
fi

PX4_WORLDS_DIR="$PX4_DIR/Tools/simulation/gz/worlds"
PX4_MODELS_DIR="$PX4_DIR/Tools/simulation/gz/models"
PX4_AIRFRAMES_DIR="$PX4_DIR/ROMFS/px4fmu_common/init.d-posix/airframes"
PX4_AIRFRAMES_CMAKE="$PX4_AIRFRAMES_DIR/CMakeLists.txt"

link_one() {
    local src="$1" dst="$2"
    if [[ -L "$dst" ]]; then
        if [[ "$(readlink -f "$dst")" == "$(readlink -f "$src")" ]]; then
            echo "[link_gz_assets] up to date: $dst"
            return
        fi
        echo "[link_gz_assets] replacing stale symlink: $dst"
        rm "$dst"
    elif [[ -e "$dst" ]]; then
        echo "ERROR: $dst already exists and is not a symlink we manage. Move it aside first."
        exit 1
    fi
    ln -s "$src" "$dst"
    echo "[link_gz_assets] linked: $dst -> $src"
}

for world in "$ASSETS_DIR"/worlds/*.sdf; do
    link_one "$world" "$PX4_WORLDS_DIR/$(basename "$world")"
done

for model in "$ASSETS_DIR"/models/*/; do
    model_name="$(basename "$model")"
    link_one "${model%/}" "$PX4_MODELS_DIR/$model_name"
done

# Airframe startup scripts (ROMFS/px4fmu_common/init.d-posix/airframes/) need
# two things: the file itself (symlinked, like worlds/models above) and an
# entry in that directory's CMakeLists.txt px4_add_romfs_files() list so the
# file actually gets copied into the SITL ROMFS image PX4 boots from — a
# `gz_<model>` make target can exist without this, but PX4 will fail at
# runtime with "no such airframe" if the entry is missing. Custom airframes
# added here use numbers in PX4's own reserved [22000, 22999] custom-model
# range (see the comment above the closing ')' in that CMakeLists.txt).
if [[ -d "$ASSETS_DIR/airframes" ]]; then
    for airframe in "$ASSETS_DIR"/airframes/*; do
        airframe_name="$(basename "$airframe")"
        link_one "$airframe" "$PX4_AIRFRAMES_DIR/$airframe_name"

        if grep -qF "$airframe_name" "$PX4_AIRFRAMES_CMAKE"; then
            echo "[link_gz_assets] CMakeLists.txt already lists $airframe_name"
        else
            # Insert before the closing ')' of the px4_add_romfs_files(...) list.
            sed -i "0,/^)/s/^)/\t${airframe_name}\n)/" "$PX4_AIRFRAMES_CMAKE"
            echo "[link_gz_assets] added $airframe_name to CMakeLists.txt"
        fi
    done
fi

echo "[link_gz_assets] done."
