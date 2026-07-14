"""Loopback-only aiohttp dashboard with bounded latest-snapshot delivery."""

from __future__ import annotations

import asyncio
from contextlib import suppress
from pathlib import Path
from typing import Any

from .runtime import RobotRuntime

STATIC_ROOT = Path(__file__).with_name("web_static")


class SnapshotHub:
    def __init__(self, runtime: RobotRuntime) -> None:
        self.runtime = runtime
        self.clients: set[asyncio.Queue[str]] = set()
        self.task: asyncio.Task[None] | None = None

    async def start(self) -> None:
        self.task = asyncio.create_task(self._broadcast(), name="telemetry-snapshot-hub")

    async def close(self) -> None:
        if self.task:
            self.task.cancel()
            with suppress(asyncio.CancelledError):
                await self.task

    async def _broadcast(self) -> None:
        previous = ""
        while True:
            snapshot = self.runtime.snapshot_json()
            if snapshot != previous:
                previous = snapshot
                for client in tuple(self.clients):
                    if client.full():
                        with suppress(asyncio.QueueEmpty):
                            client.get_nowait()
                    with suppress(asyncio.QueueFull):
                        client.put_nowait(snapshot)
            await asyncio.sleep(0.1)


def create_app(runtime: RobotRuntime):
    try:
        from aiohttp import web
    except ImportError as exc:
        raise RuntimeError(
            "Missing aiohttp. Install the project with: python3 -m pip install -e ."
        ) from exc

    @web.middleware
    async def local_only(request: Any, handler: Any):
        hostname = request.host.split(":", 1)[0].strip("[]")
        if request.remote not in ("127.0.0.1", "::1") or hostname not in ("127.0.0.1", "localhost"):
            raise web.HTTPForbidden(text="dashboard is loopback-only")
        origin = request.headers.get("Origin")
        if origin and not (
            origin.startswith("http://127.0.0.1:") or origin.startswith("http://localhost:")
        ):
            raise web.HTTPForbidden(text="cross-origin control is not allowed")
        return await handler(request)

    app = web.Application(client_max_size=16 * 1024, middlewares=[local_only])
    hub = SnapshotHub(runtime)
    app["runtime"] = runtime
    app["hub"] = hub

    async def index(_: Any):
        return web.FileResponse(STATIC_ROOT / "index.html")

    async def snapshot(_: Any):
        return web.Response(text=runtime.snapshot_json(), content_type="application/json")

    async def json_object(request: Any) -> dict[str, Any]:
        try:
            body = await request.json()
        except Exception as exc:
            raise web.HTTPBadRequest(text="request body must be valid JSON") from exc
        if not isinstance(body, dict):
            raise web.HTTPBadRequest(text="request body must be a JSON object")
        return body

    async def action(request: Any):
        body = await json_object(request)
        name = body.get("action")
        allowed = {"arm", "disarm", "estop", "clear_estop", "clear_fault", "refresh_parameters"}
        if name not in allowed:
            raise web.HTTPBadRequest(text="unsupported action")
        if not runtime.submit(name):
            raise web.HTTPServiceUnavailable(text="runtime command queue is full")
        return web.json_response({"accepted": True})

    async def parameter(request: Any):
        body = await json_object(request)
        if not isinstance(body.get("group"), str) or not isinstance(body.get("values"), dict):
            raise web.HTTPBadRequest(text="group and values are required")
        accepted = runtime.submit(
            "set_host_input" if body["group"] == "HOST_INPUT" else "set_parameter",
            {"group": body["group"], "index": body.get("index", 0), "values": body["values"]},
        )
        if not accepted:
            raise web.HTTPServiceUnavailable(text="runtime command queue is full")
        return web.json_response({"accepted": True})

    async def websocket(request: Any):
        ws = web.WebSocketResponse(heartbeat=20.0, max_msg_size=4096)
        await ws.prepare(request)
        updates: asyncio.Queue[str] = asyncio.Queue(maxsize=1)
        hub.clients.add(updates)
        updates.put_nowait(runtime.snapshot_json())
        try:
            while not ws.closed:
                await ws.send_str(await updates.get())
        except (ConnectionResetError, asyncio.CancelledError):
            pass
        finally:
            hub.clients.discard(updates)
        return ws

    async def startup(_: Any) -> None:
        runtime.start()
        await hub.start()

    async def cleanup(_: Any) -> None:
        await hub.close()
        await asyncio.to_thread(runtime.stop)

    app.router.add_get("/", index)
    app.router.add_get("/api/snapshot", snapshot)
    app.router.add_post("/api/action", action)
    app.router.add_post("/api/parameter", parameter)
    app.router.add_get("/ws", websocket)
    app.router.add_static("/static", STATIC_ROOT)
    app.on_startup.append(startup)
    app.on_cleanup.append(cleanup)
    return app
