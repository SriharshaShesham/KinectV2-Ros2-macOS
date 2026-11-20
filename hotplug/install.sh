#!/usr/bin/env bash
# Install / remove / pause the Kinect v2 hotplug supervisor (a per-user LaunchAgent).
#   bash hotplug/install.sh install     load at login, start now
#   bash hotplug/install.sh uninstall   stop and remove
#   bash hotplug/install.sh pause       stop the managed processes (for manual runs)
#   bash hotplug/install.sh resume      hand control back to the supervisor
#   bash hotplug/install.sh status      what it sees and runs
set -euo pipefail
REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
LABEL=com.kinectv2.hotplug
PLIST="$HOME/Library/LaunchAgents/$LABEL.plist"
PYTHON="${PYTHON:-$(command -v /opt/homebrew/bin/python3 || command -v python3)}"
RUN="$HOME/.kinectv2/run"

case "${1:-}" in
  install)
    mkdir -p "$HOME/.kinectv2/log" "$RUN" "$HOME/Library/LaunchAgents"
    sed -e "s#@PYTHON@#$PYTHON#g" -e "s#@REPO@#$REPO#g" -e "s#@HOME@#$HOME#g" \
      "$REPO/hotplug/$LABEL.plist.template" > "$PLIST"
    launchctl bootout "gui/$(id -u)/$LABEL" 2>/dev/null || true
    launchctl bootstrap "gui/$(id -u)" "$PLIST"
    echo "installed: $PLIST (python: $PYTHON)"
    echo "status:    cat $RUN/hotplug-status.json"
    ;;
  uninstall)
    # bootout sends SIGTERM: the supervisor stops its children before exiting.
    launchctl bootout "gui/$(id -u)/$LABEL" 2>/dev/null || true
    rm -f "$PLIST"; echo "removed $LABEL" ;;
  pause)  mkdir -p "$RUN"; touch "$RUN/hotplug-paused"; echo "paused — manual runs are safe now" ;;
  resume) rm -f "$RUN/hotplug-paused"; echo "resumed" ;;
  status)
    launchctl print "gui/$(id -u)/$LABEL" 2>/dev/null | grep -E "state|pid" | head -3 || echo "not loaded"
    cat "$RUN/hotplug-status.json" 2>/dev/null || echo "(no status file yet)" ;;
  *) echo "usage: $0 install|uninstall|pause|resume|status"; exit 2 ;;
esac
