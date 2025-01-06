# Auto-generated MQTT message class from TOML configuration config/bot_quickstart_msgs.toml at 2025-01-05 21:49:46.353895

import json

from utils.messages.mqtt_message_base import MqttMessageBase


class ROBOT_POSE_GRID_COORDS_MSG(MqttMessageBase):
    timestamp: float = None
    x_grid: int = None
    theta_rad: float = None
    y_grid: int = None

    def __init__(self, timestamp: float = None, x_grid: int = None, theta_rad: float = None, y_grid: int = None):
        """Initialize the message class with given fields."""
        self.timestamp = timestamp
        self.x_grid = x_grid
        self.theta_rad = theta_rad
        self.y_grid = y_grid

    def convert_to_payload(self) -> str:
        """Convert the message fields to a JSON payload."""
        try:
            data = {
                'timestamp': self.timestamp,
                'x_grid': self.x_grid,
                'theta_rad': self.theta_rad,
                'y_grid': self.y_grid,
            }
            return json.dumps(data)
        except (TypeError, ValueError) as e:
            raise Exception(f'Error converting to payload: {e}')

    def convert_to_message(self, payload):
        """Convert a JSON payload to message fields."""
        try:
            data = json.loads(payload)
            if 'data' in data:
                data = data['data']
            self.timestamp = data['timestamp']
            self.x_grid = data['x_grid']
            self.theta_rad = data['theta_rad']
            self.y_grid = data['y_grid']
        except (json.JSONDecodeError, KeyError) as e:
            raise Exception(f'Error converting from payload: {e}')
