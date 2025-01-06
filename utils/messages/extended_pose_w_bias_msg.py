# Auto-generated MQTT message class from TOML configuration config/bot_quickstart_msgs.toml at 2025-01-05 21:49:46.349770

import json

from utils.messages.mqtt_message_base import MqttMessageBase


class EXTENDED_POSE_W_BIAS_MSG(MqttMessageBase):
    timestamp: float = None
    r_a_b_x: float = None
    r_a_b_y: float = None
    r_a_b_z: float = None
    v_a_b_x: float = None
    v_a_b_y: float = None
    v_a_b_z: float = None
    phi_a_b_x: float = None
    phi_a_b_y: float = None
    phi_a_b_z: float = None
    bias_gyro_x: float = None
    bias_gyro_y: float = None
    bias_gyro_z: float = None
    bias_accel_x: float = None
    bias_accel_y: float = None
    bias_accel_z: float = None

    def __init__(self, timestamp: float = None, r_a_b_x: float = None, r_a_b_y: float = None, r_a_b_z: float = None, v_a_b_x: float = None, v_a_b_y: float = None, v_a_b_z: float = None, phi_a_b_x: float = None, phi_a_b_y: float = None, phi_a_b_z: float = None, bias_gyro_x: float = None, bias_gyro_y: float = None, bias_gyro_z: float = None, bias_accel_x: float = None, bias_accel_y: float = None, bias_accel_z: float = None):
        """Initialize the message class with given fields."""
        self.timestamp = timestamp
        self.r_a_b_x = r_a_b_x
        self.r_a_b_y = r_a_b_y
        self.r_a_b_z = r_a_b_z
        self.v_a_b_x = v_a_b_x
        self.v_a_b_y = v_a_b_y
        self.v_a_b_z = v_a_b_z
        self.phi_a_b_x = phi_a_b_x
        self.phi_a_b_y = phi_a_b_y
        self.phi_a_b_z = phi_a_b_z
        self.bias_gyro_x = bias_gyro_x
        self.bias_gyro_y = bias_gyro_y
        self.bias_gyro_z = bias_gyro_z
        self.bias_accel_x = bias_accel_x
        self.bias_accel_y = bias_accel_y
        self.bias_accel_z = bias_accel_z

    def convert_to_payload(self) -> str:
        """Convert the message fields to a JSON payload."""
        try:
            data = {
                'timestamp': self.timestamp,
                'r_a_b_x': self.r_a_b_x,
                'r_a_b_y': self.r_a_b_y,
                'r_a_b_z': self.r_a_b_z,
                'v_a_b_x': self.v_a_b_x,
                'v_a_b_y': self.v_a_b_y,
                'v_a_b_z': self.v_a_b_z,
                'phi_a_b_x': self.phi_a_b_x,
                'phi_a_b_y': self.phi_a_b_y,
                'phi_a_b_z': self.phi_a_b_z,
                'bias_gyro_x': self.bias_gyro_x,
                'bias_gyro_y': self.bias_gyro_y,
                'bias_gyro_z': self.bias_gyro_z,
                'bias_accel_x': self.bias_accel_x,
                'bias_accel_y': self.bias_accel_y,
                'bias_accel_z': self.bias_accel_z,
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
            self.r_a_b_x = data['r_a_b_x']
            self.r_a_b_y = data['r_a_b_y']
            self.r_a_b_z = data['r_a_b_z']
            self.v_a_b_x = data['v_a_b_x']
            self.v_a_b_y = data['v_a_b_y']
            self.v_a_b_z = data['v_a_b_z']
            self.phi_a_b_x = data['phi_a_b_x']
            self.phi_a_b_y = data['phi_a_b_y']
            self.phi_a_b_z = data['phi_a_b_z']
            self.bias_gyro_x = data['bias_gyro_x']
            self.bias_gyro_y = data['bias_gyro_y']
            self.bias_gyro_z = data['bias_gyro_z']
            self.bias_accel_x = data['bias_accel_x']
            self.bias_accel_y = data['bias_accel_y']
            self.bias_accel_z = data['bias_accel_z']
        except (json.JSONDecodeError, KeyError) as e:
            raise Exception(f'Error converting from payload: {e}')
