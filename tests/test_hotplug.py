"""hotplug/kinect_hotplug.py: what should run, and process lifecycle."""
import os
import sys
import time

import pytest

import kinect_hotplug as hp

KINECT = dict(vid=0x045E, name="Xbox NUI Sensor", speed=3)
KINECT_USB2 = dict(vid=0x045E, name="Xbox NUI Sensor", speed=2)


def comps():
    return [hp.Component(s) for s in hp.COMPONENTS]


def test_registry_is_kinect_and_console_only():
    names = [c["name"] for c in hp.COMPONENTS]
    assert names == ["kinect", "console"]
    assert hp.COMPONENTS[1]["requires"] == ["kinect"]
    for c in hp.COMPONENTS:
        assert c["cmd"][0] == hp.PIXI and c["cwd"] == hp.REPO   # runs from this repo only


def test_desired_follows_usb_presence_and_dependencies():
    cs = comps()
    assert hp.desired_set(cs, [KINECT]) == {"kinect": True, "console": True}
    assert hp.desired_set(cs, [KINECT_USB2]) == {"kinect": False, "console": False}
    assert hp.desired_set(cs, []) == {"kinect": False, "console": False}


def test_topo_order_puts_dependencies_first():
    cs = comps()
    assert [c.name for c in hp.topo_order(cs[::-1])] == ["kinect", "console"]


@pytest.fixture
def sleeper(tmp_path, monkeypatch):
    monkeypatch.setattr(hp, "LOG_DIR", str(tmp_path))
    marker = "kinect-hotplug-test-%d" % os.getpid()
    spec = dict(name="t", detect=None, always=True, requires=[], cwd=str(tmp_path),
                cmd=[sys.executable, "-c", "import time,sys; time.sleep(60)", marker],
                stop_grace_s=2, settle_s=0, conflict_pattern=marker + "-nomatch")
    return hp.Component(spec)


def test_component_start_and_graceful_stop(sleeper):
    sleeper.start()
    assert sleeper.state == "running" and sleeper.proc.poll() is None
    pid = sleeper.proc.pid
    sleeper.stop("test")
    assert sleeper.proc is None and sleeper.state == "stopped"
    with pytest.raises(ProcessLookupError):
        os.kill(pid, 0)


def test_component_blocked_when_file_missing(sleeper):
    sleeper.spec["require_files"] = ["/nonexistent/install/setup.bash"]
    sleeper.start()
    assert sleeper.state == "blocked" and sleeper.proc is None


def test_crash_is_reaped_with_backoff(sleeper):
    sleeper.spec["cmd"] = [sys.executable, "-c", "raise SystemExit(3)"]
    sleeper.start()
    for _ in range(50):
        if sleeper.proc.poll() is not None:
            break
        time.sleep(0.05)
    sleeper.reap(desired=True)
    assert sleeper.state == "backoff" and sleeper.last_exit == 3
    assert sleeper.backoff_until > time.time() and sleeper.backoff_s == 4
    assert len(sleeper.restart_times) == 1
