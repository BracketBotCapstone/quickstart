# Auto-generated MQTT message class from TOML configuration config/bot_quickstart_msgs.toml at 2025-01-05 21:49:46.359224

import json

from utils.messages.mqtt_message_base import MqttMessageBase


class TARGET_VELOCITY_MSG(MqttMessageBase):
    timestamp: float = None
    linear_velocity_mps: float = None
    angular_velocity_radps: float = None

    def __init__(self, timestamp: float = None, linear_velocity_mps: float = None, angular_velocity_radps: float = None):
        """Initialize the message class with given fields."""
        self.timestamp = timestamp
        self.linear_velocity_mps = linear_velocity_mps
        self.angular_velocity_radps = angular_velocity_radps

    def convert_to_payload(self) -> str:
        """Convert the message fields to a JSON payload."""
        try:
            data = {
                'timestamp': self.timestamp,
                'linear_velocity_mps': self.linear_velocity_mps,
                'angular_velocity_radps': self.angular_velocity_radps,
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
            self.linear_velocity_mps = data['linear_velocity_mps']
            self.angular_velocity_radps = data['angular_velocity_radps']
        except (json.JSONDecodeError, KeyError) as e:
            raise Exception(f'Error converting from payload: {e}')
