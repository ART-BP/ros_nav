#!/usr/bin/env bash
set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PY_SCRIPT="${SCRIPT_DIR}/neupan_service_server.py"

if [[ ! -f "${PY_SCRIPT}" ]]; then
  echo "Cannot find neupan_service_server.py next to ${BASH_SOURCE[0]}" >&2
  exit 1
fi

export MPLCONFIGDIR="${MPLCONFIGDIR:-/tmp/matplotlib}"
mkdir -p "${MPLCONFIGDIR}" 2>/dev/null || true

DEPS_DIR="${NEUPAN_PYTHON_DEPS_DIR:-/tmp/neupan_python_deps}"
mkdir -p "${DEPS_DIR}" 2>/dev/null || true
for candidate in /usr/lib/python3/dist-packages/netifaces.cpython-*.so /usr/lib/python3/dist-packages/netifaces.so; do
  if [[ -f "${candidate}" && "${candidate}" != *cpython-38d* ]]; then
    ln -sf "${candidate}" "${DEPS_DIR}/$(basename "${candidate}")"
    export PYTHONPATH="${DEPS_DIR}${PYTHONPATH:+:${PYTHONPATH}}"
    break
  fi
done

if [[ -n "${NEUPAN_PYTHON:-}" ]]; then
  PYTHON_BIN="${NEUPAN_PYTHON}"
else
  CONDA_SETUP="${NEUPAN_CONDA_SETUP:-${HOME}/anaconda3/etc/profile.d/conda.sh}"
  CONDA_ENV="${NEUPAN_CONDA_ENV:-neupan38}"

  if [[ ! -f "${CONDA_SETUP}" ]]; then
    echo "Cannot find conda setup script: ${CONDA_SETUP}" >&2
    echo "Set NEUPAN_CONDA_SETUP or NEUPAN_PYTHON before launching neupan_service." >&2
    exit 1
  fi

  # shellcheck source=/dev/null
  source "${CONDA_SETUP}"
  conda activate "${CONDA_ENV}"
  PYTHON_BIN="python"
fi

if [[ "${1:-}" == "--check-import" ]]; then
  exec "${PYTHON_BIN}" -c "import sys, neupan, rospy, netifaces, rosgraph.network; print(sys.executable); print(neupan.__file__); print(rospy.__file__); print(netifaces.__file__)"
fi

exec "${PYTHON_BIN}" "${PY_SCRIPT}" "$@"
