# Auto-generated MQTT message class from TOML configuration config/bot_quickstart_msgs.toml at 2025-01-05 21:49:46.337226

import json

from utils.messages.mqtt_message_base import MqttMessageBase


class GYRO_DATA_MSG(MqttMessageBase):
    timestamp: float = None
    roll: float = None
    pitch: float = None
    yaw: float = None

    def __init__(self, timestamp: float = None, roll: float = None, pitch: float = None, yaw: float = None):
        """Initialize the message class with given fields."""
        self.timestamp = timestamp
        self.roll = roll
        self.pitch = pitch
        self.yaw = yaw

    def convert_to_payload(self) -> str:
        """Convert the message fields to a JSON payload."""
        try:
            data = {
                'timestamp': self.timestamp,
                'roll': self.roll,
                'pitch': self.pitch,
                'yaw': self.yaw,
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
            self.roll = data['roll']
            self.pitch = data['pitch']
            self.yaw = data['yaw']
        except (json.JSONDecodeError, KeyError) as e:
            raise Exception(f'Error converting from payload: {e}')
