#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_DIR="${WS_DIR:-$SCRIPT_DIR}"
ROS_SETUP="${ROS_SETUP:-/opt/ros/noetic/setup.bash}"
RUNTIME_DIR="${RUNTIME_DIR:-$WS_DIR/runtime}"
INSTALL_DIR="${INSTALL_DIR:-$RUNTIME_DIR/install}"
ARCH="$(uname -m)"
STAMP="$(date +%Y%m%d_%H%M%S)"
TARBALL="${TARBALL:-$RUNTIME_DIR/real_nav_runtime_${ARCH}_${STAMP}.tar.gz}"

if [[ ! -f "$ROS_SETUP" ]]; then
  echo "ROS setup file not found: $ROS_SETUP" >&2
  exit 1
fi

if [[ ! -d "$WS_DIR/src" ]]; then
  echo "Workspace source directory not found: $WS_DIR/src" >&2
  exit 1
fi

set +u
source "$ROS_SETUP"
set -u

cd "$WS_DIR"

rm -rf build devel "$RUNTIME_DIR"
mkdir -p "$RUNTIME_DIR"

catkin_make install \
  -DCMAKE_BUILD_TYPE=Release \
  -DCATKIN_ENABLE_TESTING=OFF \
  -DCMAKE_INSTALL_PREFIX="$INSTALL_DIR"

# Remove development-only files from the install space. Keep generated Python
# message modules and setup files because ROS tooling needs them at runtime.
rm -rf "$INSTALL_DIR/include"
rm -rf "$INSTALL_DIR/share/costmap_2d/test"
rm -rf "$INSTALL_DIR/share/costmap_2d/launch"
rm -rf "$INSTALL_DIR/share/teb_local_planner/launch"
rm -rf "$INSTALL_DIR/share/teb_local_planner/scripts"
rm -f "$INSTALL_DIR/lib/teb_local_planner/"*.py
rm -f "$INSTALL_DIR/lib/teb_local_planner/test_optim_node"

find "$INSTALL_DIR" -type f \( \
  -name '*.cpp' -o \
  -name '*.cc' -o \
  -name '*.c' -o \
  -name '*.h' -o \
  -name '*.hpp' \
\) -delete

if command -v strip >/dev/null 2>&1; then
  while IFS= read -r -d '' file_path; do
    if file "$file_path" | grep -qE 'ELF .* (executable|shared object|relocatable)'; then
      strip --strip-unneeded "$file_path" 2>/dev/null || true
    fi
  done < <(find "$INSTALL_DIR" -type f -print0)
fi

tar -czf "$TARBALL" -C "$RUNTIME_DIR" install

echo "Runtime install space: $INSTALL_DIR"
echo "Runtime archive:       $TARBALL"
