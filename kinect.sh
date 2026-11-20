#!/usr/bin/env bash
# One command to install and run the Kinect v2 stack on macOS.
#
#   ./kinect.sh                 install whatever is missing, build, start everything, open the console
#   ./kinect.sh install         only install + build (no run)
#   ./kinect.sh run             only run (assumes installed)
#   ./kinect.sh stop            stop a running stack (also one started from another terminal)
#   ./kinect.sh status          what is installed / plugged in / running
#
# Options for run:  --lan (console reachable from other devices)  --port N (default 8095)
#                   --no-browser   --no-console (publishers only)
#
# Safe to run again: every step checks first and skips what is already done.
set -euo pipefail

REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$REPO"
PORT=8095
HOST=127.0.0.1
BROWSER=1
CONSOLE=1
CMD=all

for a in "$@"; do
  case "$a" in
    install|run|stop|status|all) CMD="$a" ;;
    --lan) HOST=0.0.0.0 ;;
    --port=*) PORT="${a#--port=}" ;;
    --port) echo "use --port=N"; exit 2 ;;
    --no-browser) BROWSER=0 ;;
    --no-console) CONSOLE=0 ;;
    -h|--help) sed -n '2,15p' "$0"; exit 0 ;;
    *) echo "unknown argument: $a (try --help)"; exit 2 ;;
  esac
done

say()  { printf '\033[1;36m==>\033[0m %s\n' "$*"; }
ok()   { printf '    \033[32m✓\033[0m %s\n' "$*"; }
warn() { printf '    \033[33m!\033[0m %s\n' "$*"; }
die()  { printf '\033[31merror:\033[0m %s\n' "$*" >&2; exit 1; }

PIXI="$(command -v pixi || echo "$HOME/.pixi/bin/pixi")"
have_freenect2() {
  PKG_CONFIG_PATH="/usr/local/lib/pkgconfig:/opt/homebrew/lib/pkgconfig" \
    pkg-config --exists freenect2 2>/dev/null || [ -f /usr/local/lib/pkgconfig/freenect2.pc ]
}
kinect_usb() { python3 hotplug/kinect_hotplug.py --scan 2>/dev/null | grep -E "^kinect " || true; }
our_pids() { pgrep -f "$REPO/install/(kinect|kinect_tools|kinect_console)/" || true; }

# ---------------------------------------------------------------- install
do_install() {
  say "Checking the Mac"
  [ "$(uname)" = Darwin ] || die "this script is for macOS"
  ok "macOS $(sw_vers -productVersion) on $(uname -m)"

  if xcode-select -p >/dev/null 2>&1; then ok "Xcode Command Line Tools"
  else
    warn "Xcode Command Line Tools missing: a macOS installer window will open."
    xcode-select --install || true
    die "finish the Command Line Tools install, then run ./kinect.sh again"
  fi

  if command -v brew >/dev/null 2>&1 || [ -x /opt/homebrew/bin/brew ]; then
    eval "$(/opt/homebrew/bin/brew shellenv 2>/dev/null || brew shellenv)"
    ok "Homebrew"
  else
    say "Installing Homebrew (asks for your password)"
    /bin/bash -c "$(curl -fsSL https://raw.githubusercontent.com/Homebrew/install/HEAD/install.sh)"
    eval "$(/opt/homebrew/bin/brew shellenv)"
  fi

  if have_freenect2; then ok "libfreenect2 $(PKG_CONFIG_PATH=/usr/local/lib/pkgconfig pkg-config --modversion freenect2 2>/dev/null)"
  else
    say "Building libfreenect2 (a few minutes; asks for your password to install into /usr/local)"
    bash scripts/install_libfreenect2.sh
  fi

  if [ -x "$PIXI" ]; then ok "pixi $("$PIXI" --version | awk '{print $2}')"
  else
    say "Installing pixi"
    curl -fsSL https://pixi.sh/install.sh | bash
    PIXI="$HOME/.pixi/bin/pixi"
  fi

  say "ROS 2 environment (pinned by pixi.lock)"
  [ -f pixi.lock ] || die "pixi.lock is missing — restore it from git; a fresh solve breaks the driver"
  "$PIXI" install -e kilted --locked 2>&1 | tail -1 | sed 's/^/    /'

  say "Building the workspace"
  "$PIXI" run -e kilted build 2>&1 | grep -E "^Summary|Failed|failed" | sed 's/^/    /'
  [ -f install/setup.bash ] || die "build failed — run: pixi run -e kilted build"
  ok "kinect, kinect_tools, kinect_console built"
}

# ---------------------------------------------------------------- status
do_status() {
  say "Status"
  have_freenect2 && ok "libfreenect2 installed" || warn "libfreenect2 not installed"
  [ -x "$PIXI" ] && ok "pixi installed" || warn "pixi not installed"
  [ -d .pixi/envs/kilted ] && ok "ROS 2 environment installed" || warn "ROS 2 environment not installed"
  [ -f install/setup.bash ] && ok "workspace built" || warn "workspace not built"
  local k; k="$(kinect_usb)"
  case "$k" in
    *superspeed_ok=True*) ok "Kinect on USB at SuperSpeed" ;;
    *present=True*) warn "Kinect on USB but NOT at USB 3 speed — use a USB 3 port or power-cycle its adapter" ;;
    *) warn "Kinect not on USB" ;;
  esac
  if [ -n "$(all_pids)" ]; then ok "stack running (pids $(all_pids | tr '\n' ' '))"; else warn "stack not running"; fi
  if launchctl print "gui/$(id -u)/com.kinectv2.hotplug" >/dev/null 2>&1; then
    warn "hotplug supervisor installed — it manages the stack (bash hotplug/install.sh status)"
  fi
}

# ---------------------------------------------------------------- stop
all_pids() {   # the launcher, its nodes, the console, and the run wrapper
  { pgrep -f "$REPO/.pixi/envs/kilted/bin/ros2 launch kinect" ; our_pids ;
    pgrep -f "pixi run -e kilted bash -c .*ros2 launch kinect kinect.launch.py" ; } 2>/dev/null | sort -u
}
do_stop() {
  if [ -z "$(all_pids)" ]; then ok "nothing running"; return; fi
  say "Stopping the Kinect stack"
  # The launcher first: it would otherwise respawn audio_publisher.
  for sig in INT TERM KILL; do
    pkill -$sig -f "$REPO/.pixi/envs/kilted/bin/ros2 launch kinect" 2>/dev/null || true
    pkill -$sig -f "$REPO/install/" 2>/dev/null || true
    for _ in $(seq 1 16); do [ -z "$(all_pids)" ] && break; sleep 0.5; done
    [ -z "$(all_pids)" ] && break
    pkill -$sig -f "pixi run -e kilted bash -c .*ros2 launch kinect kinect.launch.py" 2>/dev/null || true
  done
  [ -z "$(all_pids)" ] && ok "stopped" || warn "still running: $(all_pids | tr '\n' ' ')"
}

# ---------------------------------------------------------------- run
do_run() {
  [ -f install/setup.bash ] || die "not installed yet — run ./kinect.sh (or ./kinect.sh install)"

  say "Checking the Kinect"
  local k; k="$(kinect_usb)"
  case "$k" in
    *superspeed_ok=True*) ok "Kinect v2 on USB at SuperSpeed" ;;
    *present=True*) die "Kinect is on USB but not at USB 3 speed. Use a USB 3 port, or pull the adapter's power plug for ~10 s." ;;
    *) die "No Kinect v2 on USB. Plug the adapter's power in and its USB cable into a USB 3 port (check: ./kinect.sh status)." ;;
  esac

  if launchctl print "gui/$(id -u)/com.kinectv2.hotplug" >/dev/null 2>&1 \
     && [ ! -f "$HOME/.kinectv2/run/hotplug-paused" ]; then
    die "the hotplug supervisor is running the stack already. Use it (http://127.0.0.1:$PORT), or pause it: bash hotplug/install.sh pause"
  fi
  if [ -n "$(all_pids)" ]; then
    die "the stack is already running (./kinect.sh stop to stop it). Console: http://127.0.0.1:$PORT"
  fi
  local other; other="$(pgrep -fl "kinect_node|Protonect" | grep -v "$REPO" || true)"
  [ -z "$other" ] || die "another program holds the Kinect (only one can stream):
$other
Stop it first (another workspace's driver, Protonect, or its own hotplug service)."
  if lsof -nP -iTCP:"$PORT" -sTCP:LISTEN >/dev/null 2>&1; then
    die "port $PORT is busy — pick another: ./kinect.sh --port=8096"
  fi

  mkdir -p log
  say "Starting (logs in log/kinect.log and log/console.log; Ctrl-C stops everything)"
  # Publishers and the console are separate processes: open more consoles or
  # viewers any time (see README, "Subscribers").
  KINECT_HOST="$HOST" KINECT_PORT="$PORT" KINECT_CONSOLE="$CONSOLE" KINECT_BROWSER="$BROWSER" \
  exec "$PIXI" run -e kilted bash -c '
    source install/setup.bash
    # Job control: each background job gets its own process group with normal
    # signal handling (without it, non-interactive bash starts them with SIGINT
    # ignored and they would survive Ctrl-C).
    set -m
    ros2 launch kinect kinect.launch.py > log/kinect.log 2>&1 &
    LAUNCH=$!
    CONSOLE_PID=
    stop_group() {   # SIGINT -> 10 s -> SIGTERM -> 5 s -> SIGKILL, whole process group
      local pg=$1
      kill -INT -- -"$pg" 2>/dev/null || return 0
      for i in $(seq 1 20); do kill -0 -- -"$pg" 2>/dev/null || return 0; sleep 0.5; done
      kill -TERM -- -"$pg" 2>/dev/null
      for i in $(seq 1 10); do kill -0 -- -"$pg" 2>/dev/null || return 0; sleep 0.5; done
      kill -KILL -- -"$pg" 2>/dev/null
    }
    cleanup() {
      trap - INT TERM EXIT
      echo; echo "==> Stopping"
      [ -n "$CONSOLE_PID" ] && stop_group "$CONSOLE_PID"
      stop_group "$LAUNCH"
      wait 2>/dev/null
      echo "    stopped"
    }
    trap cleanup INT TERM EXIT

    printf "    waiting for the driver"
    for i in $(seq 1 240); do                      # the first start after an install can take minutes
      grep -q "Kinect v2 started" log/kinect.log 2>/dev/null && break
      if grep -qE "FATAL|No Kinect v2 device found|Failed to open" log/kinect.log 2>/dev/null; then
        echo; echo "    the driver could not open the Kinect:"; grep -E "FATAL|Failed" log/kinect.log | tail -3; exit 1
      fi
      kill -0 $LAUNCH 2>/dev/null || { echo; echo "    the launch exited — see log/kinect.log"; exit 1; }
      printf "."; sleep 1
    done
    echo; grep -q "Kinect v2 started" log/kinect.log || { echo "    no start after 4 min — see log/kinect.log"; exit 1; }
    echo "    ✓ driver streaming (color, depth, IR, mic array, point cloud)"

    if [ "$KINECT_CONSOLE" = 1 ]; then
      ros2 run kinect_console console --host "$KINECT_HOST" --port "$KINECT_PORT" > log/console.log 2>&1 &
      CONSOLE_PID=$!
      for i in $(seq 1 30); do curl -s -o /dev/null "http://127.0.0.1:$KINECT_PORT/" && break; sleep 0.5; done
      URL="http://127.0.0.1:$KINECT_PORT/"
      echo "    ✓ console at $URL"
      [ "$KINECT_HOST" = 0.0.0.0 ] && echo "      on the LAN: http://$(ipconfig getifaddr en0 2>/dev/null || hostname):$KINECT_PORT/"
      [ "$KINECT_BROWSER" = 1 ] && open "$URL"
    fi
    echo
    echo "    Topics: ros2 topic list   ·   more viewers: see README, Running"
    echo "    Press Ctrl-C to stop."
    wait $LAUNCH
  '
}

case "$CMD" in
  install) do_install ;;
  run) do_run ;;
  stop) do_stop ;;
  status) do_status ;;
  all) do_install; echo; do_run ;;
esac
