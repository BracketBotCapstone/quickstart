# Auto-generated MQTT message class from TOML configuration config/bot_quickstart_msgs.toml at 2025-01-05 21:49:46.352768

import json

from utils.messages.mqtt_message_base import MqttMessageBase


class ROBOT_EXTENDED_POSE_MSG(MqttMessageBase):
    timestamp: float = None
    r_a_b_x_m: float = None
    r_a_b_y_m: float = None
    r_a_b_z_m: float = None
    v_a_b_x_mps: float = None
    v_a_b_y_mps: float = None
    v_a_b_z_mps: float = None
    phi_a_b_x_rad: float = None
    phi_a_b_y_rad: float = None
    phi_a_b_z_rad: float = None

    def __init__(self, timestamp: float = None, r_a_b_x_m: float = None, r_a_b_y_m: float = None, r_a_b_z_m: float = None, v_a_b_x_mps: float = None, v_a_b_y_mps: float = None, v_a_b_z_mps: float = None, phi_a_b_x_rad: float = None, phi_a_b_y_rad: float = None, phi_a_b_z_rad: float = None):
        """Initialize the message class with given fields."""
        self.timestamp = timestamp
        self.r_a_b_x_m = r_a_b_x_m
        self.r_a_b_y_m = r_a_b_y_m
        self.r_a_b_z_m = r_a_b_z_m
        self.v_a_b_x_mps = v_a_b_x_mps
        self.v_a_b_y_mps = v_a_b_y_mps
        self.v_a_b_z_mps = v_a_b_z_mps
        self.phi_a_b_x_rad = phi_a_b_x_rad
        self.phi_a_b_y_rad = phi_a_b_y_rad
        self.phi_a_b_z_rad = phi_a_b_z_rad

    def convert_to_payload(self) -> str:
        """Convert the message fields to a JSON payload."""
        try:
            data = {
                'timestamp': self.timestamp,
                'r_a_b_x_m': self.r_a_b_x_m,
                'r_a_b_y_m': self.r_a_b_y_m,
                'r_a_b_z_m': self.r_a_b_z_m,
                'v_a_b_x_mps': self.v_a_b_x_mps,
                'v_a_b_y_mps': self.v_a_b_y_mps,
                'v_a_b_z_mps': self.v_a_b_z_mps,
                'phi_a_b_x_rad': self.phi_a_b_x_rad,
                'phi_a_b_y_rad': self.phi_a_b_y_rad,
                'phi_a_b_z_rad': self.phi_a_b_z_rad,
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
            self.r_a_b_x_m = data['r_a_b_x_m']
            self.r_a_b_y_m = data['r_a_b_y_m']
            self.r_a_b_z_m = data['r_a_b_z_m']
            self.v_a_b_x_mps = data['v_a_b_x_mps']
            self.v_a_b_y_mps = data['v_a_b_y_mps']
            self.v_a_b_z_mps = data['v_a_b_z_mps']
            self.phi_a_b_x_rad = data['phi_a_b_x_rad']
            self.phi_a_b_y_rad = data['phi_a_b_y_rad']
            self.phi_a_b_z_rad = data['phi_a_b_z_rad']
        except (json.JSONDecodeError, KeyError) as e:
            raise Exception(f'Error converting from payload: {e}')
