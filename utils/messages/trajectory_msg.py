from .mqtt_message_base import MqttMessageBase
import numpy as np
import json

class TRAJECTORY_MSG(MqttMessageBase):
    timestamp: float = None
    trajectory: dict = None

    def __init__(self, timestamp: float = None, trajectory: dict = None):
        self.timestamp = timestamp
        self.trajectory = trajectory

    def convert_to_payload(self) -> str:
        try:
            data = {
                'timestamp': self.timestamp,
                'trajectory': self.trajectory,
            }
            return json.dumps(data)
        except (TypeError, ValueError) as e:
            raise Exception(f'Error converting to payload: {e}')

    def convert_to_message(self, payload):
        try:
            data = json.loads(payload)
            self.timestamp = data['timestamp']
            self.trajectory = data['trajectory']
        except (json.JSONDecodeError, KeyError) as e:
            raise Exception(f'Error converting from payload: {e}')
