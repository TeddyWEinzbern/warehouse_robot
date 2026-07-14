"""PyInstaller-safe entry point for the host runtime and local GUI."""

from robot_control.cli import main


if __name__ == "__main__":
    raise SystemExit(main())
