import json


class MqttMessageBase(object):
    def __init__(self):
        self.default_msg_dict: dict = None

    def convert_to_payload(self) -> str:
        try:
            return json.dumps(self.default_msg_dict)
        except Exception as e:
            raise e
        
    def convert_to_message(self, payload):
        try:
            self.default_msg_dict = json.loads(payload)
        except Exception as e:
            raise e
