from __future__ import annotations

import argparse
import subprocess
import sys
import time
import urllib.error
import urllib.request
from pathlib import Path


TACTILE_DIR = Path(__file__).resolve().parent
PYTHON_TOOLS_DIR = TACTILE_DIR / "python_tools"
CAMERA_SERVICE_DIR = PYTHON_TOOLS_DIR / "python_tools"
UI_PROJECT = PYTHON_TOOLS_DIR / "UR3Demo.UI" / "UR3Demo.UI.csproj"


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Step1: start XIMEA camera FastAPI service and UR3Demo upper-computer UI."
    )
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=8001)
    parser.add_argument("--python", default=sys.executable, help="Python executable for uvicorn.")
    parser.add_argument("--dotnet", default="dotnet", help="dotnet executable.")
    parser.add_argument("--wait-timeout", type=float, default=20.0)
    parser.add_argument("--skip-health-wait", action="store_true")
    return parser.parse_args()


def health_url(host: str, port: int) -> str:
    return f"http://{host}:{port}/health"


def is_healthy(host: str, port: int, timeout: float = 1.0) -> bool:
    try:
        with urllib.request.urlopen(health_url(host, port), timeout=timeout) as response:
            return response.status == 200
    except (urllib.error.URLError, TimeoutError, OSError):
        return False


def wait_for_health(host: str, port: int, timeout_s: float) -> bool:
    deadline = time.time() + timeout_s
    while time.time() < deadline:
        if is_healthy(host, port):
            return True
        time.sleep(0.5)
    return False


def powershell_command(command: str) -> list[str]:
    return [
        "powershell",
        "-NoExit",
        "-ExecutionPolicy",
        "Bypass",
        "-Command",
        command,
    ]


def start_camera_service(args: argparse.Namespace) -> subprocess.Popen:
    if not CAMERA_SERVICE_DIR.exists():
        raise FileNotFoundError(f"Camera service folder not found: {CAMERA_SERVICE_DIR}")

    command = (
        f"Set-Location -LiteralPath '{CAMERA_SERVICE_DIR}'; "
        f"& '{args.python}' -m uvicorn camera_service:app --host {args.host} --port {args.port}"
    )
    print("[Step1] starting camera service:")
    print("       ", command)
    return subprocess.Popen(
        powershell_command(command),
        creationflags=subprocess.CREATE_NEW_CONSOLE,
    )


def start_upper_pc(args: argparse.Namespace) -> subprocess.Popen:
    if not UI_PROJECT.exists():
        raise FileNotFoundError(f"UR3Demo.UI project not found: {UI_PROJECT}")

    command = (
        f"Set-Location -LiteralPath '{PYTHON_TOOLS_DIR}'; "
        f"& '{args.dotnet}' run --project '.\\UR3Demo.UI\\UR3Demo.UI.csproj'"
    )
    print("[Step1] starting upper-computer UI:")
    print("       ", command)
    return subprocess.Popen(
        powershell_command(command),
        creationflags=subprocess.CREATE_NEW_CONSOLE,
    )


def main() -> None:
    args = parse_args()

    print("[Step1] camera health:", health_url(args.host, args.port))
    if is_healthy(args.host, args.port):
        print("[Step1] camera service is already running.")
    else:
        start_camera_service(args)
        if not args.skip_health_wait:
            print("[Step1] waiting for camera service...")
            if wait_for_health(args.host, args.port, args.wait_timeout):
                print("[Step1] camera service is ready.")
            else:
                print("[Step1] warning: camera service did not answer /health before timeout.")
                print("[Step1] check the camera service PowerShell window for errors.")

    start_upper_pc(args)
    print("[Step1] done. Two windows should now be open: camera service and UR3Demo.UI.")


if __name__ == "__main__":
    main()
