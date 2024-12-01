from .mqtt_message_base import MqttMessageBase
import numpy as np
import json

class OCCUPANCY_GRID_MSG(MqttMessageBase):
    timestamp: float = None
    width: int = None
    flattened_grid_list: list = None

    def __init__(self, timestamp: float = None, width: int = None, flattened_grid_list: list = None):
        self.timestamp = timestamp
        self.width = width
        self.flattened_grid_list = flattened_grid_list

    def convert_to_payload(self) -> str:
        try:
            data = {
                'timestamp': self.timestamp,
                'width': self.width,
                'flattened_grid_list': self.flattened_grid_list,
            }
            return json.dumps(data)
        except (TypeError, ValueError) as e:
            raise Exception(f'Error converting to payload: {e}')

    def convert_to_message(self, payload):
        try:
            data = json.loads(payload)
            self.timestamp = data['timestamp']
            self.width = data['width']
            self.flattened_grid_list = data['flattened_grid_list']
        except (json.JSONDecodeError, KeyError) as e:
            raise Exception(f'Error converting from payload: {e}')
