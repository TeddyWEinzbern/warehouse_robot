# Build with: python3 -m PyInstaller warehouse_robot_gui.spec
from PyInstaller.utils.hooks import collect_submodules


a = Analysis(
    ["scripts/warehouse_robot_entry.py"],
    pathex=["scripts"],
    binaries=[],
    datas=[("scripts/robot_control/web_static", "robot_control/web_static")],
    hiddenimports=collect_submodules("aiohttp"),
    hookspath=[],
    hooksconfig={},
    runtime_hooks=[],
    excludes=[],
    noarchive=False,
)
pyz = PYZ(a.pure)
exe = EXE(
    pyz,
    a.scripts,
    a.binaries,
    a.datas,
    [],
    name="warehouse-robot",
    debug=False,
    bootloader_ignore_signals=False,
    strip=False,
    upx=True,
    console=True,
)
