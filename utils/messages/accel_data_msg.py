from .mqtt_message_base import MqttMessageBase
import numpy as np
import json

class ACCEL_DATA_MSG(MqttMessageBase):
    timestamp: float = None
    x: float = None
    y: float = None
    z: float = None

    def __init__(self, timestamp: float = None, x: float = None, y: float = None, z: float = None):
        self.timestamp = timestamp
        self.x = x
        self.y = y
        self.z = z

    def convert_to_payload(self) -> str:
        try:
            data = {
                'timestamp': self.timestamp,
                'x': self.x,
                'y': self.y,
                'z': self.z,
            }
            return json.dumps(data)
        except (TypeError, ValueError) as e:
            raise Exception(f'Error converting to payload: {e}')

    def convert_to_message(self, payload):
        try:
            data = json.loads(payload)
            self.timestamp = data['timestamp']
            self.x = data['x']
            self.y = data['y']
            self.z = data['z']
        except (json.JSONDecodeError, KeyError) as e:
            raise Exception(f'Error converting from payload: {e}')
