#!/usr/bin/env python3
import os
import sys

import libtmux


def build_config_and_messages():
    """
    Constructs the command to build the configuration and messages for the bot.

    Returns
    -------
        str: The command to execute for building the configuration and messages.
    """
    # Build the config and messages
    return "cd ~/quickstart && CONFIG=bot_quickstart CONFIG_MSGS=bot_quickstart_msgs make build"

def create_dev_environment():
    """
    Sets up a development environment using tmux with a 3x3 grid of panes,
    each running a specific node of the bracket bot capstone project.
    """
    # Create a new server instance
    server = libtmux.Server()

    # Define session name
    SESSION_NAME = "dev-environment"

    # Kill existing session if it exists
    try:
        existing_session = server.get_session(SESSION_NAME)
        if existing_session:
            existing_session.kill()
    except:
        pass

    try:
        # Start a new session
        session = server.new_session(
            session_name=SESSION_NAME,
            window_name="main",
            detach=True,
        )

        # Get the first window
        main_window = session.windows[0]
        top_right_pane = main_window.attached_pane

        top_right_pane.send_keys(build_config_and_messages())

        # Split window vertically into two panes
        top_left_pane = top_right_pane.split_window(vertical=True)

        # Split both panes horizontally to create 2x2 grid
        middle_left_pane = top_left_pane.split_window(vertical=False)
        middle_right_pane = top_right_pane.split_window(vertical=False)

        # Split once more to create 3x3 grid
        bottom_left_pane = middle_left_pane.split_window(vertical=False)
        bottom_right_pane = middle_right_pane.split_window(vertical=False)

        bottom_bottom_pane = bottom_left_pane.split_window(vertical=False)

        # Ensure even layout
        main_window.select_layout('tiled')

        # Enable mouse support in tmux
        main_window.set_window_option('mouse', 'on')

        # Send commands to each pane in 3x2 grid
        top_right_pane.send_keys('cd ~/quickstart && python3 nodes/sensors/imu/main.py')
        middle_right_pane.send_keys('cd ~/quickstart && python3 nodes/control/main.py')
        bottom_right_pane.send_keys('cd ~/quickstart && python3 nodes/localization/localization_imu_enc.py')
        top_left_pane.send_keys('cd ~/quickstart && python3 nodes/planning/main.py')
        middle_left_pane.send_keys('cd ~/quickstart && python3 nodes/mapping/mapping_wavemap.py')
        # bottom_left_pane.send_keys('cd ~/quickstart && python3 nodes/rerun_viewer/main.py')
        bottom_left_pane.send_keys('cd ~/quickstart && python3 nodes/autodeploy_legs/node_autodeploy_legs.py')
        bottom_bottom_pane.send_keys('cd ~/quickstart/examples && python3 -m http.server 8080')

        # bottom_left_pane.send_keys('cd ~/quickstart && python3 examples/game_interface.py')

        print(f"Tmux session '{SESSION_NAME}' created successfully!")

        # Attach to session if not already in tmux
        if not os.environ.get('TMUX'):
            os.system(f"tmux attach -t {SESSION_NAME}")
        else:
            print(f"To attach to this session later, use: tmux attach -t {SESSION_NAME}")

    except Exception as e:
        print(f"Error creating tmux session: {e}")
        sys.exit(1)

if __name__ == "__main__":
    create_dev_environment()