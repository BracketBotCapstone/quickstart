# Adds the lib directory to the Python path
import sys
import os
sys.path.append(os.path.join(os.path.dirname(__file__), '..'))

import rerun as rr
from lib.camera import StereoCamera
import time

def main():
    # Scale down the images to 1/4 of their original size to reduce bandwidth
    camera = StereoCamera(scale=0.25)
    
    # Initialize Rerun with a descriptive name
    rr.init("stereo_camera_stream", spawn=False)
    
    # Replace with your Computer's IP address if needed
    # if you want to find you computers address do: "arp -a" in the terminal
    rr.connect_grpc(f"rerun+http://<YOUR_COMPUTER_IP>:9876/proxy")
    
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
