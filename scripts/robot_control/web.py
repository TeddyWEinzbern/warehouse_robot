"""Loopback-only aiohttp dashboard with bounded latest-snapshot delivery."""

from __future__ import annotations

import asyncio
from contextlib import suppress
import json
from pathlib import Path
import socket
import sys
from typing import Any

from .runtime import RobotRuntime

STATIC_ROOT = Path(__file__).with_name("web_static")


def reserve_dashboard_socket(host: str, port: int) -> socket.socket:
    """Bind the dashboard listener before any hardware runtime is started."""
    listener = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    try:
        if sys.platform == "win32":
            # Windows SO_REUSEADDR permits overlapping listeners and has
            # indeterminate dispatch semantics.  That would defeat the port
            # reservation that protects the serial runtime from a second UI.
            exclusive = getattr(socket, "SO_EXCLUSIVEADDRUSE", None)
            if exclusive is not None:
                listener.setsockopt(socket.SOL_SOCKET, exclusive, 1)
        else:
            listener.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        listener.bind((host, port))
        listener.listen(socket.SOMAXCONN)
        listener.setblocking(False)
        return listener
    except BaseException:
        listener.close()
        raise


class SnapshotHub:
    def __init__(self, runtime: RobotRuntime) -> None:
        self.runtime = runtime
        self.clients: set[asyncio.Queue[str | None]] = set()
        self.task: asyncio.Task[None] | None = None
        self._reported_error_events: set[tuple[Any, str]] = set()
        self._reported_last_error = ""
        self._reported_fatal_error = ""

    async def start(self) -> None:
        if self.task is not None:
            return
        self.task = asyncio.create_task(
            self._broadcast(), name="critical-status-snapshot-hub"
        )

    async def close(self) -> None:
        if self.task:
            self.task.cancel()
            # _broadcast already reports unexpected failures to stderr; cleanup
            # must remain reliable even if that background task has failed.
            with suppress(asyncio.CancelledError, Exception):
                await self.task
            self.task = None

    def _report_runtime_errors(self, snapshot: str) -> None:
        try:
            status = json.loads(snapshot)
        except (TypeError, ValueError):
            return
        if not isinstance(status, dict):
            return

        fatal_error = status.get("fatal_error")
        if not isinstance(fatal_error, str):
            fatal_error = ""
        event_messages: set[str] = set()
        events = status.get("events", [])
        if isinstance(events, list):
            for event in events:
                if not isinstance(event, dict) or event.get("level") != "error":
                    continue
                message = event.get("message")
                if not isinstance(message, str) or not message:
                    continue
                event_messages.add(message)
                key = (event.get("time"), message)
                if key not in self._reported_error_events:
                    self._reported_error_events.add(key)
                    if message != fatal_error:
                        print(f"Runtime error: {message}", file=sys.stderr, flush=True)

        last_error = status.get("last_error")
        if not isinstance(last_error, str):
            last_error = ""
        if not last_error:
            self._reported_last_error = ""
        elif last_error != self._reported_last_error:
            self._reported_last_error = last_error
            if last_error not in event_messages:
                print(f"Runtime error: {last_error}", file=sys.stderr, flush=True)

        if fatal_error and fatal_error != self._reported_fatal_error:
            self._reported_fatal_error = fatal_error
            print(f"Runtime fatal error: {fatal_error}", file=sys.stderr, flush=True)

    async def _broadcast(self) -> None:
        previous = ""
        try:
            while True:
                snapshot = self.runtime.snapshot_json()
                if snapshot != previous:
                    previous = snapshot
                    self._report_runtime_errors(snapshot)
                    for client in tuple(self.clients):
                        if client.full():
                            with suppress(asyncio.QueueEmpty):
                                client.get_nowait()
                        with suppress(asyncio.QueueFull):
                            client.put_nowait(snapshot)
                await asyncio.sleep(0.1)
        except asyncio.CancelledError:
            raise
        except Exception as exc:
            print(
                f"Dashboard background task failed: {exc}",
                file=sys.stderr,
                flush=True,
            )
            raise


def create_app(runtime: RobotRuntime):
    try:
        from aiohttp import WSCloseCode, web
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
    websockets: dict[Any, asyncio.Queue[str | None]] = {}
    app["runtime"] = runtime
    app["hub"] = hub
    app["websockets"] = websockets

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
        allowed = {"arm", "disarm", "estop", "clear_estop", "clear_fault"}
        if name not in allowed:
            raise web.HTTPBadRequest(text="unsupported action")
        if not runtime.submit(name):
            raise web.HTTPServiceUnavailable(text="runtime command queue is full")
        return web.json_response({"accepted": True})

    async def websocket(request: Any):
        ws = web.WebSocketResponse(heartbeat=20.0, max_msg_size=4096)
        await ws.prepare(request)
        updates: asyncio.Queue[str | None] = asyncio.Queue(maxsize=1)
        hub.clients.add(updates)
        websockets[ws] = updates
        updates.put_nowait(runtime.snapshot_json())
        try:
            while not ws.closed:
                update = await updates.get()
                if update is None:
                    break
                await ws.send_str(update)
        except (ConnectionResetError, RuntimeError, asyncio.CancelledError):
            pass
        finally:
            hub.clients.discard(updates)
            websockets.pop(ws, None)
        return ws

    async def startup(_: Any) -> None:
        await hub.start()
        try:
            runtime.start()
        except Exception:
            # Thread startup can fail after partially starting the runtime.  An
            # aiohttp startup failure does not run normal shutdown callbacks.
            with suppress(Exception):
                await asyncio.to_thread(runtime.stop)
            await hub.close()
            raise

    async def shutdown(_: Any) -> None:
        sessions = tuple(websockets.items())

        async def close_websocket(ws: Any) -> None:
            # A peer is allowed to ignore the close handshake.  Bound that wait
            # so graceful shutdown cannot turn into another hardware-stop delay.
            with suppress(asyncio.TimeoutError):
                await asyncio.wait_for(
                    ws.close(
                        code=WSCloseCode.GOING_AWAY,
                        message=b"Dashboard shutting down",
                    ),
                    timeout=0.1,
                )

        closing = [asyncio.create_task(close_websocket(ws)) for ws, _ in sessions]
        # Let close() mark each response closed and emit its close frame before
        # waking the outbound-only handler.  Otherwise the handler can return
        # first and leave aiohttp waiting for the peer until shutdown_timeout.
        if closing:
            await asyncio.sleep(0)
        for _, updates in sessions:
            # Stop the broadcaster from replacing the shutdown sentinel in this
            # bounded latest-value queue while the handler is being released.
            hub.clients.discard(updates)
            if updates.full():
                with suppress(asyncio.QueueEmpty):
                    updates.get_nowait()
            with suppress(asyncio.QueueFull):
                updates.put_nowait(None)

        # aiohttp waits for active requests before on_cleanup.  Stop the robot
        # here so a connected websocket cannot defer hardware shutdown by the
        # server's default 60-second graceful-shutdown timeout.
        await asyncio.to_thread(runtime.stop)
        if closing:
            await asyncio.gather(*closing, return_exceptions=True)

    async def cleanup(_: Any) -> None:
        await hub.close()

    app.router.add_get("/", index)
    app.router.add_get("/api/snapshot", snapshot)
    app.router.add_post("/api/action", action)
    app.router.add_get("/ws", websocket)
    app.router.add_static("/static", STATIC_ROOT)
    app.on_startup.append(startup)
    app.on_shutdown.append(shutdown)
    app.on_cleanup.append(cleanup)
    return app
