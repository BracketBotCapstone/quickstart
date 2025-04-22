import os, subprocess, re, yaml
from pathlib import Path

def find_rerun_ip_port():
    try:
        # Get devices from network
        devices = [(line.split(' ')[0], re.search(r'\(([0-9\.]+)\)', line).group(1)) 
                  for line in subprocess.check_output(['arp', '-a']).decode('utf-8').splitlines() 
                  if re.search(r'\(([0-9\.]+)\)', line)]
        
        # Display devices
        print("Found the following devices on your network:")
        for i, (hostname, ip) in enumerate(devices, 1):
            print(f"{i}. {hostname} ({ip})")
        
        # Get user selection
        while not (choice := input("\nEnter the number of your computer: ")).isdigit() or not 1 <= int(choice) <= len(devices):
            print(f"Please enter a number between 1 and {len(devices)}")
        selected_ip = devices[int(choice)-1][1]
        
        # Get port
        port = input("\nEnter the Rerun Visualizer TCP port number (default: 9876): ").strip() or "9876"
        while not port.isdigit() or not 1 <= int(port) <= 65535:
            print("Port must be between 1 and 65535")
            port = input("Enter port: ").strip() or "9876"
        
        # Save config to current directory
        config_path = Path("rerun_config.yaml")
        yaml.dump({"ip": selected_ip, "port": int(port)}, open(config_path, 'w'))
        
        print(f"\nConfiguration saved to {config_path}")
        print(f"IP: {selected_ip}")
        print(f"Port: {port}")
        
    except Exception as e:
        print(f"An error occurred: {e}")

# if __name__ == "__main__":
#     find_rerun_ip_port()
