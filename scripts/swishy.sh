#!/usr/bin/env bash
set -eo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE="$(cd -- "${SCRIPT_DIR}/.." && pwd)"
ROS_SETUP="/opt/ros/jazzy/setup.bash"

usage() {
  cat <<'EOF'
Usage: scripts/swishy.sh [--slam | --map] [--glass] [--mode MODE] [--loop] [--no-teleop] [--no-eyes]
       scripts/swishy.sh --stop

Opens one Terminator window with a pane per part of the stack; each pane
prints the manual command it runs. Gazebo runs in its own partition
(GZ_PARTITION, default "swishy"), so another simulation can run alongside.

  --slam        Build a new map while navigating (SLAM Toolbox mapping + Nav2). Default.
  --map         Navigate on the saved map in maps/ (SLAM Toolbox localization + Nav2).
  --glass       Glass-walled tank (fish_world_glass.sdf), to watch from outside.
  --mode MODE   Start a showcase mode once Nav2 is up:
                showcase | patrol | dance | explorer | interactive | none (default)
  --loop        Repeat the showcase mode.
  --no-teleop   Skip the keyboard teleop pane.
  --no-eyes     Skip the left/right eye camera viewers.
  --stop        Stop everything this script started, including a stray Gazebo server.

Save a map built with --slam (overwrites maps/serialized.*):
  ros2 service call /slam_toolbox/serialize_map slam_toolbox/srv/SerializePoseGraph \
    "{filename: '$PWD/src/fishy_fish_navigation/maps/serialized'}"
EOF
}

# Every process started from a pane inherits SWISHY_LAUNCHER=1 (a Gazebo
# server whose launch died too), so --stop and the start check touch only
# this launcher's processes, never other simulations or your own terminals.
swishy_pids() {
  grep -lzx "SWISHY_LAUNCHER=1" /proc/[0-9]*/environ 2>/dev/null \
    | cut -d/ -f3 | grep -vx "$$" || true
}

nav="slam"
world="fish_world.sdf"
mode="none"
loop="false"
teleop=true
eyes=true

while (($#)); do
  case "$1" in
    --slam) nav="slam" ;;
    --map) nav="map" ;;
    --glass) world="fish_world_glass.sdf" ;;
    --mode) shift; mode="${1:-}" ;;
    --loop) loop="true" ;;
    --no-teleop) teleop=false ;;
    --no-eyes) eyes=false ;;
    --stop)
      # Ctrl-C first so the launches shut down cleanly, then anything left
      # over: a Gazebo server orphaned by a closed window keeps simulating
      # and makes the next run stutter.
      for pid in $(swishy_pids); do
        tr '\0' ' ' < /proc/"${pid}"/cmdline 2>/dev/null | grep -q "bin/ros2 " && kill -INT "${pid}" 2>/dev/null
      done
      for _ in $(seq 1 30); do [[ -z "$(swishy_pids)" ]] && break; sleep 1; done
      for pid in $(swishy_pids); do kill -TERM "${pid}" 2>/dev/null || true; done
      pkill -f "terminator -u -g .*/swishy\..*/terminator.conf" || true
      echo "Swishy stopped."
      exit 0 ;;
    -h|--help) usage; exit 0 ;;
    *) echo "Unknown option: $1" >&2; usage >&2; exit 2 ;;
  esac
  shift
done

case "${mode}" in
  showcase|patrol|dance|explorer|interactive|none) ;;
  *) echo "Unknown mode: ${mode}" >&2; usage >&2; exit 2 ;;
esac

if ! command -v terminator >/dev/null; then
  echo "Terminator is not installed (sudo apt install terminator)." >&2
  echo "Run the commands from the README's manual section instead." >&2
  exit 1
fi
if [[ ! -r "${WORKSPACE}/install/setup.bash" ]]; then
  echo "Build the workspace first: colcon build --symlink-install" >&2
  exit 1
fi
if [[ -n "$(swishy_pids)" ]]; then
  echo "Swishy is already running; stop it first: scripts/swishy.sh --stop" >&2
  exit 1
fi

# Its own Gazebo partition (and the caller's ROS domain), so Swishy runs
# beside another simulation without either seeing the other's topics.
gz_partition="${GZ_PARTITION:-swishy}"
ros_domain="${ROS_DOMAIN_ID:-0}"

run_dir="$(mktemp -d "${TMPDIR:-/tmp}/swishy.XXXXXX")"

# pane NAME WAIT_FOR COMMAND: writes a script that sources ROS, waits for the
# stack to reach WAIT_FOR, prints the manual command, runs it and keeps the
# pane open afterwards.
pane() {
  local name="$1" wait_for="$2" command="$3"
  cat > "${run_dir}/${name}.sh" <<EOF
#!/usr/bin/env bash
source "${ROS_SETUP}"
source "${WORKSPACE}/install/setup.bash"
cd "${WORKSPACE}"
export DISPLAY="\${DISPLAY:-:0}"
export GZ_PARTITION="${gz_partition}"
export ROS_DOMAIN_ID="${ros_domain}"
export SWISHY_LAUNCHER=1
case "${wait_for}" in
  sim)  echo "Waiting for Gazebo..."
        until ros2 topic list 2>/dev/null | grep -qx /scan; do sleep 2; done ;;
  nav2) echo "Waiting for Nav2..."
        until ros2 lifecycle get /bt_navigator 2>/dev/null | grep -q '^active'; do sleep 3; done ;;
esac
printf '\n\$ %s\n\n' "${command}"
${command}
unset SWISHY_LAUNCHER
exec bash
EOF
  chmod +x "${run_dir}/${name}.sh"
}

pane gazebo none "ros2 launch fishy_fish_navigation spawn_robot.launch.py world:=${world}"
pane nav sim "ros2 launch fishy_fish_navigation navigation_with_${nav}.launch.py"
if [[ "${mode}" == "none" ]]; then
  pane mode nav2 "echo Pick a mode: ros2 launch fishy_fish_navigation_py showcase_navigation.launch.py mode:=patrol"
else
  pane mode nav2 "ros2 launch fishy_fish_navigation_py showcase_navigation.launch.py mode:=${mode} loop:=${loop}"
fi
if ${teleop}; then
  pane teleop sim "ros2 run teleop_twist_keyboard teleop_twist_keyboard"
else
  pane teleop none "echo Teleop disabled"
fi
if ${eyes}; then
  pane eyes sim "ros2 run rqt_image_view rqt_image_view /left_eye/image & ros2 run rqt_image_view rqt_image_view /right_eye/image"
else
  pane eyes none "echo Eye cameras disabled"
fi

# Gazebo | navigation on the left, showcase | teleop | eyes on the right.
cat > "${run_dir}/terminator.conf" <<EOF
[global_config]
[keybindings]
[profiles]
  [[default]]
    scrollback_lines = 5000
[layouts]
  [[swishy]]
    [[[window0]]]
      type = Window
      parent = ""
      size = 1600, 950
      title = Swishy
    [[[root]]]
      type = HPaned
      parent = window0
      position = 800
    [[[left]]]
      type = VPaned
      parent = root
      order = 0
      position = 475
    [[[right]]]
      type = VPaned
      parent = root
      order = 1
      position = 400
    [[[right_bottom]]]
      type = VPaned
      parent = right
      order = 1
      position = 275
    [[[gazebo]]]
      type = Terminal
      parent = left
      order = 0
      title = Gazebo
      command = ${run_dir}/gazebo.sh
    [[[nav]]]
      type = Terminal
      parent = left
      order = 1
      title = ${nav^^} + Nav2 + RViz
      command = ${run_dir}/nav.sh
    [[[mode]]]
      type = Terminal
      parent = right
      order = 0
      title = Showcase mode
      command = ${run_dir}/mode.sh
    [[[teleop]]]
      type = Terminal
      parent = right_bottom
      order = 0
      title = "Teleop - click here and use i j k l"
      command = ${run_dir}/teleop.sh
    [[[eyes]]]
      type = Terminal
      parent = right_bottom
      order = 1
      title = Eye cameras
      command = ${run_dir}/eyes.sh
EOF

echo "Starting Swishy (${nav}, mode ${mode}, GZ_PARTITION=${gz_partition}, ROS_DOMAIN_ID=${ros_domain})."
echo "Stop with: scripts/swishy.sh --stop"
# -u: without it a running Terminator takes over the request over D-Bus and
# ignores this config, so no window opens.
terminator -u -g "${run_dir}/terminator.conf" -l swishy >/dev/null 2>&1 &
disown
