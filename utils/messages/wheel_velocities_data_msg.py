from .mqtt_message_base import MqttMessageBase
import numpy as np
import json

class WHEEL_VELOCITIES_DATA_MSG(MqttMessageBase):
    timestamp: float = None
    left_vel_mps: float = None
    right_vel_mps: float = None

    def __init__(self, timestamp: float = None, left_vel_mps: float = None, right_vel_mps: float = None):
        self.timestamp = timestamp
        self.left_vel_mps = left_vel_mps
        self.right_vel_mps = right_vel_mps

    def convert_to_payload(self) -> str:
        try:
            data = {
                'timestamp': self.timestamp,
                'left_vel_mps': self.left_vel_mps,
                'right_vel_mps': self.right_vel_mps,
            }
            return json.dumps(data)
        except (TypeError, ValueError) as e:
            raise Exception(f'Error converting to payload: {e}')

    def convert_to_message(self, payload):
        try:
            data = json.loads(payload)
            self.timestamp = data['timestamp']
            self.left_vel_mps = data['left_vel_mps']
            self.right_vel_mps = data['right_vel_mps']
        except (json.JSONDecodeError, KeyError) as e:
            raise Exception(f'Error converting from payload: {e}')
