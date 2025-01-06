# Auto-generated MQTT message class from TOML configuration config/bot_quickstart_msgs.toml at 2025-01-05 21:49:46.337482

import json

from utils.messages.mqtt_message_base import MqttMessageBase


class RAW_IMU_DATA_MSG(MqttMessageBase):
    timestamp: float = None
    accel_x: float = None
    accel_y: float = None
    accel_z: float = None
    gyro_x: float = None
    gyro_y: float = None
    gyro_z: float = None

    def __init__(self, timestamp: float = None, accel_x: float = None, accel_y: float = None, accel_z: float = None, gyro_x: float = None, gyro_y: float = None, gyro_z: float = None):
        """Initialize the message class with given fields."""
        self.timestamp = timestamp
        self.accel_x = accel_x
        self.accel_y = accel_y
        self.accel_z = accel_z
        self.gyro_x = gyro_x
        self.gyro_y = gyro_y
        self.gyro_z = gyro_z

    def convert_to_payload(self) -> str:
        """Convert the message fields to a JSON payload."""
        try:
            data = {
                'timestamp': self.timestamp,
                'accel_x': self.accel_x,
                'accel_y': self.accel_y,
                'accel_z': self.accel_z,
                'gyro_x': self.gyro_x,
                'gyro_y': self.gyro_y,
                'gyro_z': self.gyro_z,
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
            self.accel_x = data['accel_x']
            self.accel_y = data['accel_y']
            self.accel_z = data['accel_z']
            self.gyro_x = data['gyro_x']
            self.gyro_y = data['gyro_y']
            self.gyro_z = data['gyro_z']
        except (json.JSONDecodeError, KeyError) as e:
            raise Exception(f'Error converting from payload: {e}')
