from .mqtt_message_base import MqttMessageBase
import numpy as np
import json

class GYRO_DATA_MSG(MqttMessageBase):
    timestamp: float = None
    roll: float = None
    pitch: float = None
    yaw: float = None

    def __init__(self, timestamp: float = None, roll: float = None, pitch: float = None, yaw: float = None):
        self.timestamp = timestamp
        self.roll = roll
        self.pitch = pitch
        self.yaw = yaw

    def convert_to_payload(self) -> str:
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
        try:
            data = json.loads(payload)
            self.timestamp = data['timestamp']
            self.roll = data['roll']
            self.pitch = data['pitch']
            self.yaw = data['yaw']
        except (json.JSONDecodeError, KeyError) as e:
            raise Exception(f'Error converting from payload: {e}')
