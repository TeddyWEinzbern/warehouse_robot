import asyncio
from contextlib import redirect_stderr
import io
import json
import socket
import sys
import unittest
from unittest.mock import MagicMock, patch

from aiohttp import ClientSession, WSMsgType, web

from robot_control.web import create_app
from robot_control.web import SnapshotHub, reserve_dashboard_socket


class StaticRuntime:
    def __init__(self, snapshot: str = "{}"):
        self.snapshot = snapshot

    def snapshot_json(self) -> str:
        return self.snapshot


class DashboardSocketTests(unittest.TestCase):
    def test_reservation_rejects_an_existing_listener(self):
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as occupied:
            occupied.bind(("127.0.0.1", 0))
            occupied.listen(1)
            with self.assertRaises(OSError):
                reserve_dashboard_socket("127.0.0.1", occupied.getsockname()[1])

    def test_successful_reservation_is_a_nonblocking_listener(self):
        listener = reserve_dashboard_socket("127.0.0.1", 0)
        try:
            self.assertFalse(listener.getblocking())
            self.assertGreater(listener.getsockname()[1], 0)
        finally:
            listener.close()

    def test_windows_uses_exclusive_address_not_reuse_address(self):
        listener = MagicMock()
        exclusive = 0x7FFF
        with (
            patch("robot_control.web.socket.socket", return_value=listener),
            patch("robot_control.web.sys.platform", "win32"),
            patch.object(socket, "SO_EXCLUSIVEADDRUSE", exclusive, create=True),
        ):
            self.assertIs(reserve_dashboard_socket("127.0.0.1", 8765), listener)

        listener.setsockopt.assert_called_once_with(socket.SOL_SOCKET, exclusive, 1)
        self.assertNotIn(
            (socket.SOL_SOCKET, socket.SO_REUSEADDR, 1),
            [call.args for call in listener.setsockopt.call_args_list],
        )

    @unittest.skipIf(sys.platform == "win32", "POSIX socket option test")
    def test_posix_retains_reuse_address(self):
        listener = MagicMock()
        with patch("robot_control.web.socket.socket", return_value=listener):
            self.assertIs(reserve_dashboard_socket("127.0.0.1", 8765), listener)
        listener.setsockopt.assert_called_once_with(
            socket.SOL_SOCKET, socket.SO_REUSEADDR, 1
        )


class RuntimeErrorReportingTests(unittest.TestCase):
    def test_snapshot_errors_are_printed_once_to_terminal(self):
        snapshot = json.dumps(
            {
                "events": [
                    {"time": 1.25, "level": "error", "message": "serial write failed"}
                ],
                "last_error": "serial write failed",
                "fatal_error": "",
            }
        )
        hub = SnapshotHub(StaticRuntime(snapshot))
        output = io.StringIO()
        with redirect_stderr(output):
            hub._report_runtime_errors(snapshot)
            hub._report_runtime_errors(snapshot)
        self.assertEqual(output.getvalue().count("serial write failed"), 1)

    def test_last_and_fatal_errors_without_events_are_visible(self):
        hub = SnapshotHub(StaticRuntime())
        output = io.StringIO()
        with redirect_stderr(output):
            hub._report_runtime_errors(
                json.dumps(
                    {
                        "events": [],
                        "last_error": "connection attempt failed",
                        "fatal_error": "attempt limit reached",
                    }
                )
            )
        self.assertIn("Runtime error: connection attempt failed", output.getvalue())
        self.assertIn("Runtime fatal error: attempt limit reached", output.getvalue())

    def test_fatal_event_keeps_explicit_fatal_terminal_label(self):
        fatal = "Stopped after 3 consecutive serial failures"
        snapshot = json.dumps(
            {
                "events": [{"time": 2.5, "level": "error", "message": fatal}],
                "last_error": "serial write failed",
                "fatal_error": fatal,
            }
        )
        hub = SnapshotHub(StaticRuntime(snapshot))
        output = io.StringIO()
        with redirect_stderr(output):
            hub._report_runtime_errors(snapshot)
            hub._report_runtime_errors(snapshot)
        self.assertEqual(output.getvalue().count(f"Runtime fatal error: {fatal}"), 1)
        self.assertNotIn(f"Runtime error: {fatal}", output.getvalue())


class BackgroundTaskTests(unittest.IsolatedAsyncioTestCase):
    async def test_background_failure_is_printed_and_cleanup_remains_safe(self):
        class BrokenRuntime:
            def snapshot_json(self):
                raise RuntimeError("snapshot exploded")

        hub = SnapshotHub(BrokenRuntime())
        output = io.StringIO()
        with redirect_stderr(output):
            await hub.start()
            await asyncio.sleep(0)
            await hub.close()
        self.assertIn("Dashboard background task failed: snapshot exploded", output.getvalue())
        self.assertIsNone(hub.task)

    async def test_partial_runtime_start_is_stopped_when_app_startup_fails(self):
        class PartialStartRuntime:
            def __init__(self):
                self.stops = 0

            def start(self):
                raise RuntimeError("thread start failed after partial startup")

            def stop(self):
                self.stops += 1

            def snapshot_json(self):
                return "{}"

        runtime = PartialStartRuntime()
        app = create_app(runtime)
        runner = web.AppRunner(app)
        with self.assertRaisesRegex(RuntimeError, "partial startup"):
            await runner.setup()
        self.assertEqual(runtime.stops, 1)
        self.assertIsNone(app["hub"].task)


class WebsocketShutdownTests(unittest.IsolatedAsyncioTestCase):
    async def test_active_websocket_cannot_delay_runtime_stop(self):
        class LifecycleRuntime:
            def __init__(self):
                self.starts = 0
                self.stops = 0

            def start(self):
                self.starts += 1

            def stop(self):
                self.stops += 1

            def snapshot_json(self):
                return "{}"

            def submit(self, *_args):
                return True

        runtime = LifecycleRuntime()
        runner = web.AppRunner(create_app(runtime), shutdown_timeout=0.5)
        await runner.setup()
        site = web.TCPSite(runner, "127.0.0.1", 0)
        await site.start()
        port = site._server.sockets[0].getsockname()[1]
        session = ClientSession()
        cleaned_up = False
        try:
            ws = await session.ws_connect(f"http://127.0.0.1:{port}/ws")
            self.assertEqual((await ws.receive()).data, "{}")

            # Without an on_shutdown websocket close, aiohttp waits roughly
            # twice shutdown_timeout before it reaches runtime.stop in cleanup.
            await asyncio.wait_for(runner.cleanup(), timeout=0.4)
            cleaned_up = True
            self.assertEqual(runtime.starts, 1)
            self.assertEqual(runtime.stops, 1)
            close_message = await asyncio.wait_for(ws.receive(), timeout=0.2)
            self.assertIn(
                close_message.type,
                (WSMsgType.CLOSE, WSMsgType.CLOSED, WSMsgType.CLOSING),
            )
        finally:
            await session.close()
            if not cleaned_up:
                await runner.cleanup()


if __name__ == "__main__":
    unittest.main()
