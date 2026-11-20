"""kinect_console.server with a fake ROS bridge: fan-out to many browsers,
lazy subscribe/unsubscribe, audio over WebSocket."""
import asyncio

import pytest
from aiohttp.test_utils import TestClient, TestServer

from kinect_console import server, usb


class FakeBridge:
    def __init__(self, on_frame, on_audio):
        self.on_frame, self.on_audio = on_frame, on_audio
        self.video_refs, self.audio_refs = {}, 0
        self.method = None

    def acquire_video(self, src):
        self.video_refs[src] = self.video_refs.get(src, 0) + 1

    def release_video(self, src):
        self.video_refs[src] -= 1

    def acquire_audio(self):
        self.audio_refs += 1

    def release_audio(self):
        self.audio_refs -= 1

    def depth_at(self, src, x, y):
        return 1234.0 if src == "depth" else None

    def set_doa_method(self, m):
        self.method = m
        return True


def run(coro):
    return asyncio.run(coro)


async def make_client(monkeypatch):
    monkeypatch.setattr(usb, "kinect_status", lambda: {"present": True, "superspeed": True,
                                                       "speed": "5 Gb/s", "serial": "X"})
    holder = {}

    def factory(on_frame, on_audio):
        holder["b"] = FakeBridge(on_frame, on_audio)
        return holder["b"]

    client = TestClient(TestServer(server.make_app(bridge_factory=factory)))
    await client.start_server()
    return client, holder["b"]


async def read_one_frame(resp):
    buf = b""
    while b"\r\n\r\n" not in buf or len(buf) < 200:
        buf += await resp.content.read(256)
        if b"JPEGDATA" in buf:
            return buf
    return buf


def test_index_and_status(monkeypatch):
    async def go():
        client, _ = await make_client(monkeypatch)
        try:
            r = await client.get("/")
            assert r.status == 200 and "KINECT V2 CONSOLE" in await r.text()
            assert (await client.get("/static/app.js")).status == 200
            s = await (await client.get("/api/status")).json()
            assert set(s["streams"]) == set(server.SOURCES)
        finally:
            await client.close()
    run(go())


def test_one_subscription_fans_out_to_many_viewers(monkeypatch):
    async def go():
        client, bridge = await make_client(monkeypatch)
        try:
            viewers = [await client.get("/stream?src=depth") for _ in range(5)]
            await asyncio.sleep(0.05)
            assert bridge.video_refs["depth"] == 5          # each viewer counted
            hub = client.server.app["hubs"]["depth"]
            assert hub.viewers == 5
            bridge.on_frame("depth", b"JPEGDATA" * 30)      # one frame from ROS...
            frames = await asyncio.gather(*(read_one_frame(v) for v in viewers))
            assert all(b"JPEGDATA" in f for f in frames)    # ...reaches all five
            for v in viewers:
                v.close()
            await asyncio.sleep(0.2)
            assert bridge.video_refs["depth"] == 0          # last viewer left -> released
        finally:
            await client.close()
    run(go())


def test_bad_source_rejected(monkeypatch):
    async def go():
        client, _ = await make_client(monkeypatch)
        try:
            assert (await client.get("/stream?src=thermal")).status == 400
        finally:
            await client.close()
    run(go())


def test_snapshot_and_depth_at(monkeypatch):
    async def go():
        client, bridge = await make_client(monkeypatch)
        try:
            client.server.app["hubs"]["ir"].push(b"\xff\xd8IRFRAME")
            r = await client.get("/snapshot?src=ir")
            assert r.status == 200 and await r.read() == b"\xff\xd8IRFRAME"
            d = await (await client.get("/api/depth_at?src=depth&x=0.5&y=0.5")).json()
            assert d["mm"] == 1234.0
            assert (await client.get("/api/depth_at?src=depth")).status == 400
        finally:
            await client.close()
    run(go())


def test_websocket_audio_refcount_and_method(monkeypatch):
    async def go():
        client, bridge = await make_client(monkeypatch)
        try:
            a = await client.ws_connect("/ws")
            b = await client.ws_connect("/ws")
            assert (await a.receive_json())["type"] == "status"
            await b.receive_json()
            await a.send_json({"type": "audio", "on": True})
            await b.send_json({"type": "audio", "on": True})
            await asyncio.sleep(0.05)
            assert bridge.audio_refs == 2
            bridge.on_audio({"levels": [0.1, 0.1, 0.1, 0.1], "doa": 12.0})
            for ws in (a, b):
                while True:
                    m = await ws.receive_json(timeout=2)
                    if m["type"] == "audio":
                        assert m["doa"] == 12.0
                        break
            await a.send_json({"type": "doa_method", "method": "gcc"})
            while (m := await a.receive_json(timeout=2))["type"] != "doa_method":
                pass
            assert m["ok"] and bridge.method == "gcc"
            await a.close()
            await b.send_json({"type": "audio", "on": False})
            await asyncio.sleep(0.1)
            assert bridge.audio_refs == 0
            await b.close()
        finally:
            await client.close()
    run(go())
