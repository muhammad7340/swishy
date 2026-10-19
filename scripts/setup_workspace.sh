#!/usr/bin/env bash
set -eo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE="$(cd -- "${SCRIPT_DIR}/.." && pwd)"
PACKAGE_LIST="${SCRIPT_DIR}/ros-jazzy-runtime-packages.txt"
ROS_SETUP="/opt/ros/jazzy/setup.bash"

usage() {
  cat <<'EOF'
Usage: scripts/setup_workspace.sh [--check | --install] [--no-build]

  --check     Audit dependencies and linked libraries without changing the system.
  --install   Refresh apt, align critical ROS packages, run rosdep, and build.
  --no-build  Skip colcon build when used with --install.

The default mode is --check.
EOF
}

mode="check"
build_workspace=true

while (($#)); do
  case "$1" in
    --check) mode="check" ;;
    --install) mode="install" ;;
    --no-build) build_workspace=false ;;
    -h|--help) usage; exit 0 ;;
    *) echo "Unknown option: $1" >&2; usage >&2; exit 2 ;;
  esac
  shift
done

if [[ ! -r "${ROS_SETUP}" ]]; then
  echo "ROS 2 Jazzy was not found at ${ROS_SETUP}." >&2
  exit 1
fi

# shellcheck disable=SC1091
source "${ROS_SETUP}"
set -u

mapfile -t critical_packages < <(
  sed -e 's/[[:space:]]*#.*$//' -e '/^[[:space:]]*$/d' "${PACKAGE_LIST}"
)

audit_packages() {
  local package installed candidate
  local failures=0

  echo "Checking critical ROS package versions..."
  for package in "${critical_packages[@]}"; do
    installed="$(dpkg-query -W -f='${Version}' "${package}" 2>/dev/null || true)"
    candidate="$(apt-cache policy "${package}" 2>/dev/null | awk '/Candidate:/ {print $2; exit}')"

    if [[ -z "${installed}" ]]; then
      printf '  MISSING  %s\n' "${package}"
      failures=$((failures + 1))
    elif [[ -n "${candidate}" && "${candidate}" != "(none)" && "${installed}" != "${candidate}" ]]; then
      printf '  STALE    %s (%s; candidate %s)\n' "${package}" "${installed}" "${candidate}"
      failures=$((failures + 1))
    fi
  done

  if ((failures == 0)); then
    echo "  OK: all critical packages match the current apt candidates."
  fi

  return "${failures}"
}

audit_libraries() {
  local library output
  local failures=0
  local libraries=(
    /opt/ros/jazzy/lib/libcontroller_manager.so
    /opt/ros/jazzy/lib/libcontroller_manager_msgs__rosidl_typesupport_fastrtps_cpp.so
    /opt/ros/jazzy/lib/librl_lib.so
  )

  echo "Checking runtime library links..."
  for library in "${libraries[@]}"; do
    [[ -e "${library}" ]] || continue
    output="$(ldd -r "${library}" 2>&1 | grep -E 'not found|undefined symbol' || true)"
    if [[ -n "${output}" ]]; then
      printf '  BROKEN   %s\n%s\n' "${library}" "${output}"
      failures=$((failures + 1))
    fi
  done

  if ((failures == 0)); then
    echo "  OK: no unresolved runtime symbols found."
  fi

  return "${failures}"
}

audit_rosdep() {
  echo "Checking dependencies declared in package.xml..."
  if ! command -v rosdep >/dev/null 2>&1; then
    echo "  MISSING  rosdep"
    return 1
  fi

  if rosdep check --from-paths "${WORKSPACE}/src" --ignore-src --rosdistro jazzy; then
    echo "  OK: package.xml dependencies are satisfied."
  else
    return 1
  fi
}

run_audit() {
  local failed=0
  audit_packages || failed=1
  audit_libraries || failed=1
  audit_rosdep || failed=1

  if ((failed)); then
    echo
    echo "Environment check failed. Run: scripts/setup_workspace.sh --install" >&2
    return 1
  fi
}

if [[ "${mode}" == "check" ]]; then
  run_audit
  exit
fi

echo "Refreshing apt package metadata..."
if ! sudo apt-get update; then
  cat >&2 <<'EOF'

apt update failed. Disable or repair the repository named in the error, then
run this script again. Continuing with stale package metadata could recreate
the binary-version mismatch this script is intended to prevent.
EOF
  exit 1
fi

echo "Aligning compatibility-critical ROS 2 Jazzy packages..."
sudo apt-get install -y "${critical_packages[@]}"

echo "Installing dependencies declared by package.xml..."
rosdep install --from-paths "${WORKSPACE}/src" --ignore-src -r -y --rosdistro jazzy

if [[ "${build_workspace}" == true ]]; then
  echo "Building the workspace..."
  cd "${WORKSPACE}"
  colcon build --symlink-install
fi

echo "Recording the resolved package versions..."
mkdir -p "${WORKSPACE}/install"
dpkg-query -W -f='${Package}\t${Version}\n' "${critical_packages[@]}" \
  > "${WORKSPACE}/install/ros-package-versions.txt"

run_audit

cat <<EOF

Setup complete. In each new terminal, run:
  source /opt/ros/jazzy/setup.bash
  source ${WORKSPACE}/install/setup.bash
EOF
