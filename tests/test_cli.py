import argparse
from contextlib import redirect_stderr, redirect_stdout
import io
from pathlib import Path
import socket
import unittest
from unittest.mock import Mock, patch

from robot_control import cli
from robot_control.transport import SerialConnectionError


def run_args(port: str, web_port: int = 8765) -> argparse.Namespace:
    return argparse.Namespace(
        command="run",
        port=port,
        baud=9600,
        web_port=web_port,
        no_gamepad=True,
        reconnect_attempts=2,
    )


class SerialDeviceValidationTests(unittest.TestCase):
    def test_macos_missing_dev_prefix_has_actionable_error(self):
        with patch(
            "robot_control.cli.list_ports",
            return_value=["/dev/cu.HC-06 - HC-06 Bluetooth serial"],
        ):
            with self.assertRaisesRegex(
                SerialConnectionError,
                r"use the full path '/dev/cu\.HC-06'.*'/dev/' prefix is required",
            ):
                cli.validate_serial_device("cu.HC-06", platform="darwin")

    def test_windows_com_port_validation_is_case_insensitive(self):
        with patch(
            "robot_control.cli.list_ports",
            return_value=["COM12 - Standard Serial over Bluetooth link"],
        ):
            cli.validate_serial_device(r"\\.\com12", platform="win32")
            with self.assertRaisesRegex(SerialConnectionError, "COM13"):
                cli.validate_serial_device("COM13", platform="win32")

    def test_posix_regular_file_is_not_accepted_as_a_serial_device(self):
        regular_file = str(Path(__file__).resolve())
        with patch("robot_control.cli.list_ports", return_value=[]):
            with self.assertRaisesRegex(SerialConnectionError, "character device"):
                cli.validate_serial_device(regular_file, platform="linux")

    def test_posix_character_device_fallback_is_accepted(self):
        path = Mock()
        path.is_absolute.return_value = True
        path.is_char_device.return_value = True
        with (
            patch("robot_control.cli.list_ports", return_value=[]),
            patch("robot_control.cli.Path", return_value=path),
        ):
            cli.validate_serial_device("/dev/tty-fake", platform="linux")

    def test_invalid_device_stops_before_dashboard_or_runtime_creation(self):
        with (
            patch("robot_control.cli.list_ports", return_value=[]),
            patch("robot_control.cli.RobotRuntime") as runtime_class,
            patch("robot_control.web.reserve_dashboard_socket") as reserve,
        ):
            with self.assertRaises(SerialConnectionError):
                cli.run(run_args("cu.missing-test-device"))
        reserve.assert_called_once_with("127.0.0.1", 8765)
        reserve.return_value.close.assert_called_once_with()
        runtime_class.assert_not_called()


class DashboardStartupTests(unittest.TestCase):
    def test_parser_defaults_to_hc06_baud_and_rejects_one_way(self):
        with patch(
            "sys.argv",
            [
                "warehouse-robot",
                "run",
                "--port",
                "/dev/cu.test",
            ],
        ):
            args = cli.parse_args()
        self.assertEqual(args.baud, 9600)
        with (
            patch(
                "sys.argv",
                [
                    "warehouse-robot",
                    "run",
                    "--port",
                    "/dev/cu.test",
                    "--one-way",
                ],
            ),
            self.assertRaises(SystemExit),
        ):
            cli.parse_args()

    def test_runtime_receives_only_verified_link_options(self):
        with (
            patch("robot_control.cli.validate_serial_device"),
            patch("robot_control.cli.RobotRuntime") as runtime_class,
            patch("robot_control.web.reserve_dashboard_socket"),
            patch("robot_control.web.create_app", return_value=object()),
            patch("aiohttp.web.run_app"),
        ):
            cli.run(run_args("/dev/cu.test"))
        runtime_class.assert_called_once_with(
            "/dev/cu.test",
            9600,
            use_gamepad=False,
            maximum_reconnect_attempts=2,
        )

    def test_occupied_web_port_never_constructs_or_starts_runtime(self):
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as occupied:
            occupied.bind(("127.0.0.1", 0))
            occupied.listen(1)
            web_port = occupied.getsockname()[1]
            with (
                patch("robot_control.cli.validate_serial_device") as validate,
                patch("robot_control.cli.RobotRuntime") as runtime_class,
            ):
                with self.assertRaisesRegex(SystemExit, "runtime was not started"):
                    cli.run(run_args("/dev/cu.test", web_port))
        validate.assert_not_called()
        runtime_class.assert_not_called()

    def test_reserved_socket_is_closed_if_web_runner_fails(self):
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as probe:
            probe.bind(("127.0.0.1", 0))
            web_port = probe.getsockname()[1]

        captured = []

        def fail_web_runner(_app, *, sock):
            captured.append(sock)
            raise RuntimeError("synthetic web startup failure")

        with (
            patch("robot_control.cli.validate_serial_device"),
            patch("robot_control.cli.RobotRuntime"),
            patch("robot_control.web.create_app", return_value=object()),
            patch("aiohttp.web.run_app", side_effect=fail_web_runner),
        ):
            with self.assertRaisesRegex(RuntimeError, "synthetic web startup failure"):
                cli.run(run_args("/dev/cu.test", web_port))

        self.assertEqual(len(captured), 1)
        self.assertEqual(captured[0].fileno(), -1)


class MainErrorReportingTests(unittest.TestCase):
    def test_serial_connection_error_is_written_to_stderr(self):
        stdout = io.StringIO()
        stderr = io.StringIO()
        with (
            patch("robot_control.cli.parse_args", return_value=argparse.Namespace()),
            patch(
                "robot_control.cli.run",
                side_effect=SerialConnectionError("serial unavailable"),
            ),
            redirect_stdout(stdout),
            redirect_stderr(stderr),
        ):
            result = cli.main()
        self.assertEqual(result, 2)
        self.assertEqual(stdout.getvalue(), "")
        self.assertIn("Error: serial unavailable", stderr.getvalue())


if __name__ == "__main__":
    unittest.main()
