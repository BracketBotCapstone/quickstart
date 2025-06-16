# Adds the lib directory to the Python path
# NOTE: On some consumer routers—particularly certain models sold in the USA—
# running `arp -a` may list devices only by MAC address (or not at all) because
# the router doesn't maintain a local DNS table. If that happens, the automatic
# IP discovery in this script may return an empty list and you'll need to enter
# the viewer's IP manually.
import sys
import os
sys.path.append(os.path.join(os.path.dirname(__file__), '..'))

import rerun as rr
from lib.camera import StereoCamera

# New imports for CLI interaction and network discovery
import subprocess
import re
from typing import List, Tuple

def main():
    """Stream stereo images to a Rerun viewer with interactive connection setup.

    The script will try to connect to a Rerun viewer via gRPC. If the default
    address fails, it offers a simple interactive flow to help the user pick the
    correct IP address of the machine running the viewer. This is handy when
    cloning the repository on a new device or when the viewer's host IP changes.
    """

    # ---------------------------------------------------------------------
    # 1. Helper utilities
    # ---------------------------------------------------------------------

    def _discover_devices() -> List[Tuple[str, str]]:
        """Return `(hostname, ip)` tuples from the local ARP table.

        Hostname may be '?' (unknown) on some routers. If `arp` is not
        available, the list is empty.
        """
        try:
            out = subprocess.check_output(["arp", "-a"], text=True)
            # Each line looks like: "gateway (192.168.0.1) at ab:cd:ef ..."
            # We capture both the leading token and the IP inside parenthesis.
            return re.findall(r"([\w\-\.\?]+) \((\d+\.\d+\.\d+\.\d+)\)", out)
        except Exception:
            return []

    def _prompt_for_ip(devices: List[Tuple[str, str]]) -> str:
        print("\nI couldn't connect to a Rerun viewer.")
        if devices:
            print("Here are some devices discovered on your network that might be running the viewer:")
            for idx, (host, ip) in enumerate(devices, 1):
                label = ip if host in ("?", ip) else f"{ip} ({host})"
                print(f"  [{idx}] {label}")
            print("Enter a number to pick one of the above, or type an IP/hostname manually.")
        while True:
            choice = input("Viewer IP [localhost]: ").strip()
            if choice == "":
                return "localhost"
            if choice.isdigit():
                num = int(choice)
                if 1 <= num <= len(devices):
                    return devices[num - 1][1]  # return IP part
            return choice

    # ---------------------------------------------------------------------
    # 2. Initialize Rerun
    # ---------------------------------------------------------------------

    # NOTE: `rr.init` MUST be called before any attempt to connect or log.
    rr.init("stereo_camera_stream", spawn=False)

    # ---------------------------------------------------------------------
    # 3. Attempt to establish viewer connection (with retries)
    # ---------------------------------------------------------------------

    def _connect(ip: str) -> bool:
        url = f"rerun+http://{ip}:9876/proxy"
        try:
            rr.connect_grpc(url)
            print(f"Connected to Rerun viewer at {url}\n")
            return True
        except Exception as exc:
            print(f"Failed to connect to {url}: {exc}")
            return False

    devices = _discover_devices()
    viewer_ip = _prompt_for_ip(devices)
    connected = _connect(viewer_ip)

    def _user_confirms_visible() -> bool:
        print("\nA test message was sent to the viewer. Check your Rerun window for a run named 'stereo_camera_stream'.")
        resp = input("Do you see it? [y/N]: ").strip().lower()
        return resp in ("y", "yes")

    # Keep trying until we establish a *working* connection or the user aborts
    while True:
        if connected:
            # Emit a small test message so something appears immediately
            rr.log("status", rr.TextDocument("connection-test"))
            if _user_confirms_visible():
                break  # All good, proceed to full streaming
            print("\nLooks like the viewer didn't receive the data. Let's try a different IP.")

        # Either we weren't connected or the user said they couldn't see the stream
        devices = _discover_devices()
        viewer_ip = _prompt_for_ip(devices)
        connected = _connect(viewer_ip)

    # ---------------------------------------------------------------------
    # 4. Start camera and streaming
    # ---------------------------------------------------------------------

    # Scale down the images to 1/4 of their original size to reduce bandwidth
    camera = StereoCamera(scale=0.25)
    
    print("\nStreaming stereo images to Rerun viewer...")
    print(f"Scale factor: {camera.get_scale()}")
    print("Press Ctrl+C to stop")
    
    try:
        while True:
            left, right = camera.get_stereo()
            if left is None or right is None:
                print("Failed to capture images")
                continue
            
            rr.log("stereo/left", rr.Image(left, color_model="bgr"))
            rr.log("stereo/right", rr.Image(right, color_model="bgr"))
            
    except KeyboardInterrupt:
        print("\nStopping...")
    finally:
        camera.release()
        print("\nShutdown complete")

if __name__ == "__main__":
    main()
