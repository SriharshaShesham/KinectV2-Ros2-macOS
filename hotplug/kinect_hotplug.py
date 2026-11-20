#!/usr/bin/env python3
"""Kinect v2 hotplug supervisor (com.kinectv2.hotplug).

Polls the USB bus every POLL_S seconds and starts/stops the Kinect stack so the
sensor is plug-and-play: connect the Kinect -> `ros2 launch kinect
kinect.launch.py` comes up; unplug it -> it is stopped cleanly. Components and
their detection rules are in COMPONENTS below (add your own subscribers there).

Constraints this code exists to absorb:
  - kinect_node exits 0 when the sensor is absent, and hangs silently when it
    is unplugged mid-run: neither exit codes nor liveness can gate anything.
    USB presence decides everything.
  - launchd jobs cannot read ~/Documents until the interpreter in the plist is
    granted Full Disk Access; until then this daemon idles in a visible
    "needs-full-disk-access" state instead of half-working.
  - The Kinect only re-enumerates on a physical adapter power cycle; when that
    is the situation, say so (status + notification) instead of restart-storming.

Stdlib only; must run on any python3 (no tomllib, no pyserial).
CLI: --scan prints one detection pass and exits.
"""

import json
import os
import plistlib
import signal
import subprocess
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

HOME = os.path.expanduser("~")
LOG_DIR = HOME + "/.kinectv2/log"
RUN_DIR = HOME + "/.kinectv2/run"
STATUS = RUN_DIR + "/hotplug-status.json"
PAUSE_FLAG = RUN_DIR + "/hotplug-paused"
POLL_S = 2.0
BACKOFF_START_S, BACKOFF_CAP_S, HEALTHY_RESET_S = 2, 60, 120
QUIRK_BAD_SPEED_POLLS = 5       # not-SuperSpeed polls before flagging power cycle
QUIRK_RESTARTS, QUIRK_WINDOW_S = 3, 300
NOTIFY_GAP_S = 600

def log(msg):
    line = "%s %s" % (time.strftime("%Y-%m-%d %H:%M:%S"), msg)
    print(line, flush=True)


# This repo (the folder above hotplug/) and pixi; override with env vars.
REPO = os.environ.get("KINECT_REPO",
                      os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
PIXI = os.environ.get("PIXI", os.path.expanduser("~/.pixi/bin/pixi"))


def ros(cmd):
    """Run a ROS 2 command inside this repo's pixi env (kilted)."""
    return [PIXI, "run", "-e", "kilted", "bash", "-c",
            "source install/setup.bash && exec " + cmd]


# Fields:
#   name             component id; log = ~/.kinectv2/log/kinect.<name>.log
#   detect           None = not device-gated. Or dict(type="usb", vendor_id=<int>,
#                    product_name=<str>, require_superspeed=True)
#   always           True = desired whenever the daemon runs
#   requires         desired only while every listed component is desired;
#                    started only after each has been RUNNING >= its settle_s
#   cmd              argv list, absolute paths only (launchd PATH is minimal)
#   cwd              working directory for the process
#   stop_grace_s     SIGINT -> wait this -> SIGTERM -> wait 5 -> SIGKILL (process group)
#   settle_s         how long this must run before dependents are started
#   require_files    missing file => state "blocked", no start attempt, no crash-loop
#   conflict_pattern pgrep -f guard: a match not owned by the daemon => "conflict"
KINECT_USB = dict(type="usb", vendor_id=0x045E, product_name="Xbox NUI Sensor",
                  require_superspeed=True)

COMPONENTS = [
    dict(
        name="kinect",       # publishers: driver + audio_publisher + pointcloud_node
        detect=KINECT_USB, always=False, requires=[],
        cmd=ros("ros2 launch kinect kinect.launch.py"),
        cwd=REPO, stop_grace_s=15, settle_s=8,
        require_files=[REPO + "/install/setup.bash"],
        conflict_pattern="ros2 launch kinect kinect.launch.py",
    ),
    dict(
        name="console",      # a subscriber: the web console, up iff kinect is up
        detect=None, always=False, requires=["kinect"],
        cmd=ros("ros2 run kinect_console console"),
        cwd=REPO, stop_grace_s=5, settle_s=0,
        conflict_pattern="kinect_console/console",
    ),
]


def load_registry():
    return COMPONENTS, REPO


def scan_usb():
    """One ioreg pass -> flat list of {vid, name, speed} for the IOUSB plane."""
    out = subprocess.run(["ioreg", "-a", "-l", "-p", "IOUSB", "-w0"],
                         capture_output=True).stdout
    devices = []

    def walk(node):
        if isinstance(node, dict):
            if "idVendor" in node:
                devices.append(dict(vid=node.get("idVendor", 0),
                                    name=node.get("USB Product Name", ""),
                                    speed=node.get("Device Speed", 0)))
            for child in node.get("IORegistryEntryChildren", []):
                walk(child)
        elif isinstance(node, list):
            for child in node:
                walk(child)

    try:
        walk(plistlib.loads(out))
    except Exception as e:
        log("ioreg parse failed: %r" % e)
    return devices


def match_detect(rule, devices):
    """-> (present, superspeed_ok, matched_device_or_None)."""
    if rule is None:
        return True, True, None
    vids = rule.get("vendor_ids") or [rule["vendor_id"]]
    want_name = rule.get("product_name")
    for d in devices:
        if d["vid"] in vids and (want_name is None or d["name"] == want_name):
            ok = d["speed"] >= 3 if rule.get("require_superspeed") else True
            return True, ok, d
    return False, False, None


_last_notify = {}


def notify(key, title, msg):
    now = time.time()
    if now - _last_notify.get(key, 0) < NOTIFY_GAP_S:
        return
    _last_notify[key] = now
    log("NOTIFY %s: %s" % (title, msg))
    subprocess.run(["osascript", "-e",
                    'display notification "%s" with title "%s"' % (msg, title)],
                   capture_output=True)


class Component:
    def __init__(self, spec):
        self.spec = spec
        self.name = spec["name"]
        self.proc = None
        self.state = "stopped"   # stopped|running|backoff|blocked|conflict|needs-power-cycle
        self.started_at = 0.0
        self.restarts = 0
        self.backoff_until = 0.0
        self.backoff_s = BACKOFF_START_S
        self.last_exit = None
        self.restart_times = []  # for the kinect power-cycle heuristic

    def running_for(self):
        return time.time() - self.started_at if self.proc else 0.0

    def _conflict_pids(self):
        pat = self.spec.get("conflict_pattern")
        if not pat:
            return []
        r = subprocess.run(["pgrep", "-f", pat], capture_output=True, text=True)
        return [int(p) for p in r.stdout.split()]

    def start(self):
        for f in self.spec.get("require_files", []):
            if not os.path.exists(f):
                if self.state != "blocked":
                    log("%s blocked: missing %s" % (self.name, f))
                self.state = "blocked"
                return
        strays = self._conflict_pids()
        if strays:
            if self.state != "conflict":
                log("%s conflict: unmanaged pid(s) %s match %r — not starting"
                    % (self.name, strays, self.spec["conflict_pattern"]))
                notify("conflict-" + self.name, "Kinect hotplug",
                       "%s already running outside the service (pid %s). "
                       "Pause the supervisor for manual runs." % (self.name, strays[0]))
            self.state = "conflict"
            return
        logf = open("%s/kinect.%s.log" % (LOG_DIR, self.name), "ab")
        logf.write(("\n===== %s start %s =====\n"
                    % (time.strftime("%Y-%m-%d %H:%M:%S"), self.name)).encode())
        self.proc = subprocess.Popen(self.spec["cmd"], cwd=self.spec["cwd"],
                                     stdout=logf, stderr=logf,
                                     stdin=subprocess.DEVNULL,
                                     start_new_session=True)
        logf.close()
        self.started_at = time.time()
        self.state = "running"
        log("%s started (pid %d)" % (self.name, self.proc.pid))

    def _signal_group(self, sig):
        """killpg with macOS quirks absorbed: mid-teardown killpg can raise
        EPERM (seen live 2026-09-26); fall back to the direct child then."""
        try:
            os.killpg(self.proc.pid, sig)
            return True
        except ProcessLookupError:
            return False
        except PermissionError:
            try:
                self.proc.send_signal(sig)
                return True
            except Exception:
                return False

    def stop(self, reason):
        if not self.proc:
            self.state = "stopped"
            return
        log("%s stopping (%s)" % (self.name, reason))
        for sig, wait_s in ((signal.SIGINT, self.spec["stop_grace_s"]),
                            (signal.SIGTERM, 5), (signal.SIGKILL, 5)):
            if not self._signal_group(sig):
                break
            deadline = time.time() + wait_s
            while time.time() < deadline:
                if self.proc.poll() is not None:
                    break
                time.sleep(0.2)
            if self.proc.poll() is not None:
                self._signal_group(signal.SIGKILL)  # reap group stragglers
                break
        self.proc = None
        self.state = "stopped"
        log("%s stopped" % self.name)

    def reap(self, desired):
        """Notice an exit; schedule backoff if it should still be running."""
        if not self.proc or self.proc.poll() is None:
            return
        self.last_exit = self.proc.returncode
        ran_s = self.running_for()
        try:  # sweep orphans left in the group
            os.killpg(self.proc.pid, signal.SIGKILL)
        except (ProcessLookupError, PermissionError):
            pass
        self.proc = None
        if not desired:
            self.state = "stopped"
            return
        if ran_s >= HEALTHY_RESET_S:
            self.backoff_s = BACKOFF_START_S
        now = time.time()
        self.restarts += 1
        self.restart_times = [t for t in self.restart_times
                              if now - t < QUIRK_WINDOW_S] + [now]
        self.backoff_until = now + self.backoff_s
        log("%s exited code=%s after %.0fs — retry in %ds"
            % (self.name, self.last_exit, ran_s, self.backoff_s))
        self.backoff_s = min(self.backoff_s * 2, BACKOFF_CAP_S)
        self.state = "backoff"


def topo_order(components):
    """Registry order with dependencies hoisted before dependents."""
    order, seen = [], set()

    def visit(c):
        if c.name in seen:
            return
        seen.add(c.name)
        for r in c.spec["requires"]:
            visit(by_name[r])
        order.append(c)

    by_name = {c.name: c for c in components}
    for c in components:
        visit(c)
    return order


def desired_set(components, devices):
    """always + detected, closed under requires (dep gone => dependent gone)."""
    present = {}
    for c in components:
        p, ok, _ = match_detect(c.spec.get("detect"), devices)
        present[c.name] = p and ok
    desired = {}
    for c in topo_order(components):
        base = c.spec["always"] or (c.spec.get("detect") is not None and present[c.name])
        deps_ok = all(desired.get(r, False) for r in c.spec["requires"])
        if c.spec.get("detect") is None and not c.spec["always"] and c.spec["requires"]:
            desired[c.name] = deps_ok          # purely dependency-gated
        else:
            desired[c.name] = base and deps_ok
    return desired


def write_status(fda_ok, paused, devices_report, components):
    os.makedirs(RUN_DIR, exist_ok=True)
    data = dict(
        ts=time.strftime("%Y-%m-%dT%H:%M:%S"),
        fda="ok" if fda_ok else "needs-full-disk-access",
        paused=paused,
        devices=devices_report,
        components={c.name: dict(state=c.state,
                                 pid=c.proc.pid if c.proc else None,
                                 since=int(c.running_for()),
                                 restarts=c.restarts,
                                 last_exit=c.last_exit)
                    for c in components},
    )
    tmp = STATUS + ".tmp"
    with open(tmp, "w") as f:
        json.dump(data, f, indent=1)
    os.replace(tmp, STATUS)


def check_fda(repo):
    """launchd + TCC: a launchd job's reads of ~/Documents HANG (observed
    2026-09-26, not just EPERM) until the plist's interpreter is granted Full
    Disk Access — so probe from a child process with a timeout, never in-process."""
    try:
        r = subprocess.run(["/bin/ls", repo + "/src"],
                           capture_output=True, timeout=5)
        return r.returncode == 0
    except subprocess.TimeoutExpired:
        return False


def main():
    specs, repo = load_registry()
    components = [Component(s) for s in specs]

    if "--scan" in sys.argv:
        devs = scan_usb()
        for d in devs:
            print("vid=0x%04x speed=%d  %s" % (d["vid"], d["speed"], d["name"]))
        for c in components:
            p, ok, _ = match_detect(c.spec.get("detect"), devs)
            gate = ("always" if c.spec["always"] else
                    "requires=%s" % c.spec["requires"] if c.spec.get("detect") is None
                    else "present=%s superspeed_ok=%s" % (p, ok))
            print("%-16s %s" % (c.name, gate))
        return

    order = topo_order(components)
    stopping = {"stop": False}

    def on_signal(signum, _frame):
        stopping["stop"] = True

    signal.signal(signal.SIGTERM, on_signal)
    signal.signal(signal.SIGINT, on_signal)

    log("kinect_hotplug up; %d components: %s"
        % (len(order), ", ".join(c.name for c in order)))

    while not check_fda(repo):
        log("cannot read %s — grant Full Disk Access to the plist's python3 "
            "(System Settings > Privacy & Security), then: "
            "launchctl kickstart -k gui/$UID/com.kinectv2.hotplug" % repo)
        notify("fda", "Kinect hotplug",
               "Needs Full Disk Access for its python3 to reach the repo.")
        write_status(False, False, {}, components)
        for _ in range(30):
            if stopping["stop"]:
                return
            time.sleep(1)

    bad_speed_polls = 0
    power_cycle = False

    try:
        run_loop(components, order, stopping, bad_speed_polls, power_cycle)
    finally:
        # Reached on SIGTERM/SIGINT *and* on any crash: never orphan children —
        # an orphaned group survives the daemon and then blocks restarts as a
        # "conflict" (bitten twice on 2026-09-26).
        log("shutdown — stopping all components")
        for c in reversed(order):
            c.stop("daemon shutdown")
        write_status(True, os.path.exists(PAUSE_FLAG), {}, components)
        log("bye")


def run_loop(components, order, stopping, bad_speed_polls, power_cycle):
    while not stopping["stop"]:
        paused = os.path.exists(PAUSE_FLAG)
        devices = scan_usb()
        desired = {} if paused else desired_set(components, devices)

        # Kinect power-cycle heuristic (see module docstring)
        kin = next((c for c in components if c.name == "kinect"), None)
        devices_report = {}
        if kin is not None and kin.spec.get("detect"):
            p, ok, d = match_detect(kin.spec["detect"], devices)
            bad_speed_polls = bad_speed_polls + 1 if (p and not ok) else 0
            crash_looping = p and len(kin.restart_times) >= QUIRK_RESTARTS
            if bad_speed_polls >= QUIRK_BAD_SPEED_POLLS or crash_looping:
                if not power_cycle:
                    log("kinect needs a physical adapter power cycle "
                        "(superspeed=%s, recent restarts=%d)" % (ok, len(kin.restart_times)))
                power_cycle = True
            elif not p:
                power_cycle = False   # off the bus: a power cycle re-enumerates it
                kin.restart_times = []
            if power_cycle:
                for c in components:     # the kinect and everything that requires it
                    if c is kin or "kinect" in c.spec["requires"]:
                        desired[c.name] = False
                kin.state = "needs-power-cycle" if not kin.proc else kin.state
                notify("kinect-power", "Kinect v2",
                       "Kinect adapter needs a physical power cycle "
                       "(pull its power plug ~10 s, USB stays in).")
            devices_report["kinect"] = dict(present=p, superspeed=ok,
                                            note="needs-power-cycle" if power_cycle else "")

        # stop what is no longer wanted — dependents first
        for c in reversed(order):
            if c.proc and not desired.get(c.name, False):
                c.stop("paused" if paused else "device gone / dependency down")

        # notice exits
        for c in components:
            c.reap(desired.get(c.name, False))

        # start what is wanted — dependencies first, settle honored
        now = time.time()
        by_name = {c.name: c for c in components}
        for c in order:
            if c.proc or not desired.get(c.name, False) or now < c.backoff_until:
                continue
            deps_settled = all(
                by_name[r].proc and by_name[r].running_for() >= by_name[r].spec["settle_s"]
                for r in c.spec["requires"])
            if deps_settled:
                c.start()

        for c in components:
            if paused and not c.proc:
                c.state = "paused"
            elif not c.proc and c.state in ("paused",):
                c.state = "stopped"

        write_status(True, paused, devices_report, components)
        time.sleep(POLL_S)


if __name__ == "__main__":
    os.makedirs(LOG_DIR, exist_ok=True)
    os.makedirs(RUN_DIR, exist_ok=True)
    main()
