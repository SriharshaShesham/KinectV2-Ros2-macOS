"""Kinect v2 Console — web viewer for the kinect topics.

    ros2 run kinect_console console            # http://127.0.0.1:8095
    ros2 run kinect_console console --host 0.0.0.0 --port 8095   # reachable on the LAN

One ROS subscription per stream, fanned out to any number of browsers: each
stream keeps only its latest frame, so a slow browser skips frames instead of
buffering or slowing the others.
"""

import argparse
import asyncio
import json
import os
import time
from typing import Dict, Optional, Set

from aiohttp import WSMsgType, web

from . import usb

BOUNDARY = "kinectframe"
SOURCES = ("color", "ir", "depth", "registered", "aligned")


class FrameHub:
    """Latest-frame buffer for one stream; MJPEG clients await the next frame."""

    def __init__(self) -> None:
        self.latest: Optional[bytes] = None
        self.latest_at = 0.0
        self.seq = 0
        self.viewers = 0
        self._cond = asyncio.Condition()
        self._stamps = []

    def push(self, jpeg: bytes) -> None:
        self.latest = jpeg
        self.seq += 1
        now = time.monotonic()
        self.latest_at = now
        self._stamps = [t for t in self._stamps if now - t < 3.0] + [now]
        asyncio.ensure_future(self._notify())

    async def _notify(self) -> None:
        async with self._cond:
            self._cond.notify_all()

    async def next_frame(self, last_seq: int, timeout: float = 5.0):
        async with self._cond:
            await asyncio.wait_for(
                self._cond.wait_for(lambda: self.seq != last_seq and self.latest is not None),
                timeout)
            return self.latest, self.seq

    def fps(self) -> float:
        now = time.monotonic()
        recent = [t for t in self._stamps if now - t < 3.0]
        return round(len(recent) / 3.0, 1)


def web_dir() -> str:
    try:
        from ament_index_python.packages import get_package_share_directory
        d = os.path.join(get_package_share_directory("kinect_console"), "web")
        if os.path.isdir(d):
            return d
    except Exception:
        pass
    return os.path.join(os.path.dirname(os.path.abspath(__file__)), "web")


async def index(request):
    return web.FileResponse(os.path.join(request.app["web"], "index.html"),
                            headers={"Cache-Control": "no-store"})


async def stream(request: web.Request) -> web.StreamResponse:
    src = request.query.get("src", "color")
    if src not in SOURCES:
        raise web.HTTPBadRequest(text="src must be one of %s" % (SOURCES,))
    app = request.app
    hub: FrameHub = app["hubs"][src]
    resp = web.StreamResponse(headers={
        "Content-Type": "multipart/x-mixed-replace; boundary=" + BOUNDARY,
        "Cache-Control": "no-store"})
    await resp.prepare(request)
    hub.viewers += 1
    app["bridge"].acquire_video(src)
    seq = 0
    try:
        while True:
            try:
                jpeg, seq = await hub.next_frame(seq)
            except asyncio.TimeoutError:
                continue          # driver quiet (starting / unplugged): keep waiting
            await resp.write(b"--" + BOUNDARY.encode() + b"\r\nContent-Type: image/jpeg\r\n"
                             b"Content-Length: " + str(len(jpeg)).encode() + b"\r\n\r\n"
                             + jpeg + b"\r\n")
    except (ConnectionResetError, asyncio.CancelledError):
        pass
    finally:
        hub.viewers -= 1
        app["bridge"].release_video(src)
    return resp


async def snapshot(request):
    src = request.query.get("src", "color")
    if src not in SOURCES:
        raise web.HTTPBadRequest()
    hub = request.app["hubs"][src]
    bridge = request.app["bridge"]
    # Only a frame from the last second counts: when nobody is watching, the
    # kept frame can be minutes old.
    if hub.latest is None or time.monotonic() - hub.latest_at > 1.0:
        bridge.acquire_video(src)
        try:
            await hub.next_frame(hub.seq, timeout=5.0)
        except asyncio.TimeoutError:
            raise web.HTTPServiceUnavailable(text="no %s frame within 5 s — is the driver running?" % src)
        finally:
            bridge.release_video(src)
    return web.Response(body=hub.latest, content_type="image/jpeg",
                        headers={"Cache-Control": "no-store"})


async def depth_at(request):
    src = request.query.get("src", "depth")
    try:
        x, y = float(request.query["x"]), float(request.query["y"])
    except (KeyError, ValueError):
        raise web.HTTPBadRequest(text="x and y (0..1) required")
    mm = request.app["bridge"].depth_at(src, x, y)
    return web.json_response({"src": src, "mm": mm})


def status_payload(app) -> dict:
    hubs: Dict[str, FrameHub] = app["hubs"]
    st = app.get("usb") or {}
    return {
        "type": "status",
        "usb": st,
        "streams": {s: {"fps": h.fps(), "viewers": h.viewers} for s, h in hubs.items()},
        "clients": len(app["sockets"]),
    }


async def status(request):
    return web.json_response(status_payload(request.app))


async def ws_handler(request):
    app = request.app
    ws = web.WebSocketResponse(heartbeat=20)
    await ws.prepare(request)
    app["sockets"].add(ws)
    listening = False
    try:
        await ws.send_json(status_payload(app))
        async for msg in ws:
            if msg.type != WSMsgType.TEXT:
                continue
            try:
                m = json.loads(msg.data)
            except ValueError:
                continue
            if m.get("type") == "audio":
                want = bool(m.get("on"))
                if want and not listening:
                    app["audio_ws"].add(ws)
                    app["bridge"].acquire_audio()
                elif listening and not want:
                    app["audio_ws"].discard(ws)
                    app["bridge"].release_audio()
                listening = want
            elif m.get("type") == "doa_method" and m.get("method") in ("srp", "gcc"):
                ok = await asyncio.get_event_loop().run_in_executor(
                    None, app["bridge"].set_doa_method, m["method"])
                await ws.send_json({"type": "doa_method", "method": m["method"], "ok": ok})
    finally:
        app["sockets"].discard(ws)
        if listening:
            app["audio_ws"].discard(ws)
            app["bridge"].release_audio()
    return ws


async def broadcast(app, payload, targets) -> None:
    for ws in list(targets):
        try:
            await ws.send_json(payload)
        except Exception:
            targets.discard(ws)


async def status_loop(app):
    while True:
        try:
            app["usb"] = await asyncio.get_event_loop().run_in_executor(None, usb.kinect_status)
        except Exception as e:
            app["usb"] = {"present": False, "error": str(e)}
        await broadcast(app, status_payload(app), app["sockets"])
        await asyncio.sleep(2.0)


def make_app(bridge_factory=None) -> web.Application:
    app = web.Application()
    app["web"] = web_dir()
    app["sockets"]: Set = set()
    app["audio_ws"]: Set = set()
    app["hubs"] = {s: FrameHub() for s in SOURCES}

    async def on_startup(app):
        loop = asyncio.get_event_loop()

        def on_frame(src, jpeg):           # called on the ROS thread
            loop.call_soon_threadsafe(app["hubs"][src].push, jpeg)

        def on_audio(payload):             # called on the ROS thread
            payload["type"] = "audio"
            asyncio.run_coroutine_threadsafe(broadcast(app, payload, app["audio_ws"]), loop)

        if bridge_factory is None:
            from . import bridge
            app["bridge"] = bridge.start(on_frame, on_audio)
        else:
            app["bridge"] = bridge_factory(on_frame, on_audio)
        app["status_task"] = asyncio.ensure_future(status_loop(app))

    async def on_cleanup(app):
        app["status_task"].cancel()

    app.on_startup.append(on_startup)
    app.on_cleanup.append(on_cleanup)
    app.router.add_get("/", index)
    app.router.add_get("/stream", stream)
    app.router.add_get("/snapshot", snapshot)
    app.router.add_get("/api/depth_at", depth_at)
    app.router.add_get("/api/status", status)
    app.router.add_get("/ws", ws_handler)
    # follow_symlinks: colcon --symlink-install installs these files as symlinks
    app.router.add_static("/static/", app["web"], follow_symlinks=True)
    return app


def main(argv=None):
    p = argparse.ArgumentParser(prog="kinect_console", description=__doc__.split("\n")[0])
    p.add_argument("--host", default="127.0.0.1", help="0.0.0.0 to allow other devices on the LAN")
    p.add_argument("--port", type=int, default=8095)
    args, _ros_args = p.parse_known_args(argv)
    print("Kinect v2 Console on http://%s:%d" % (args.host, args.port), flush=True)
    web.run_app(make_app(), host=args.host, port=args.port, print=None)


if __name__ == "__main__":
    main()
