"""Auto-generated MQTT message class from TOML configuration config/bot_quickstart_msgs.toml at 2025-01-27 16:06:22.780948."""

import json

from lib.messages.mqtt_message_base import MqttMessageBase


class WHEEL_VELOCITIES_DATA_MSG(MqttMessageBase):
    """MQTT message class for WHEEL_VELOCITIES_DATA."""

    timestamp: float = None
    left_vel_mps: float = None
    right_vel_mps: float = None

    def __init__(self, timestamp: float | None = None, left_vel_mps: float | None = None, right_vel_mps: float | None = None):
        """Initialize the message class with given fields."""
        self.timestamp = timestamp
        self.left_vel_mps = left_vel_mps
        self.right_vel_mps = right_vel_mps

    def convert_to_payload(self) -> str:
        """Convert the message fields to a JSON payload."""
        try:
            data = {
                'timestamp': self.timestamp,
                'left_vel_mps': self.left_vel_mps,
                'right_vel_mps': self.right_vel_mps,
            }
            return json.dumps(data)
        except (TypeError, ValueError) as e:
            raise Exception(f'Error converting to payload: {e}') from e

    def convert_to_message(self, payload):
        """Convert a JSON payload to message fields."""
        try:
            data = json.loads(payload)
            if 'data' in data:
                data = data['data']
            self.timestamp = data['timestamp']
            self.left_vel_mps = data['left_vel_mps']
            self.right_vel_mps = data['right_vel_mps']
        except (json.JSONDecodeError, KeyError) as e:
            raise Exception(f'Error converting from payload: {e}') from e
