import base64
import json
import logging
import time
from threading import Thread

import cv2
import paho.mqtt.client as mqtt

from utils.messages.mqtt_message_base import MqttMessageBase

logging.basicConfig(level=logging.DEBUG)

class MQTTPublisher():
    def __init__(self, broker_address="localhost", topic_to_message_map: dict[str, MqttMessageBase] = None):
        """
        Initialize the MQTT Publisher with a broker address and a topic-to-message map.

        :param broker_address: Address of the MQTT broker.
        :param topic_to_message_map: Dictionary mapping topics to message classes.
        """
        self.broker_address = broker_address
        self.topic_to_message_map = topic_to_message_map
        self.client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2)
        self.unacked_publish = set()
        # self.client.on_publish = MQTTPublisher.on_publish

        self.client.user_data_set(self.unacked_publish)

    @staticmethod
    def on_publish(client, userdata, mid, reason_code, properties):
        """
        Callback for when a message is published.

        :param client: The client instance for this callback.
        :param userdata: The private user data as set in Client() or userdata_set().
        :param mid: Message ID.
        :param reason_code: Reason code for the publish (MQTTv5 only).
        :param properties: Properties associated with the publish (MQTTv5 only).
        """
        try:
            userdata.remove(mid)
        except KeyError:
            print("on_publish() is called with a mid not present in unacked_publish")
            print("This is due to an unavoidable race-condition:")
            print("* publish() return the mid of the message sent.")
            print("* mid from publish() is added to unacked_publish by the main thread")
            print("* on_publish() is called by the loop_start thread")
            print("While unlikely (because on_publish() will be called after a network round-trip),")
            print(" this is a race-condition that COULD happen")
            print("")
            print("The best solution to avoid race-condition is using the msg_info from publish()")
            print("We could also try using a list of acknowledged mid rather than removing from pending list,")
            print("but remember that mid could be re-used !")

    def connect(self, keep_alive: int = None):
        """Connect to the MQTT broker."""
        if keep_alive is not None:
            self.client.connect(self.broker_address, keepalive=keep_alive)
        else:
            self.client.connect(self.broker_address)

    def publish_dict(self, topic, data: dict):
        """
        Publish a dictionary as a JSON payload to a specified topic.

        :param topic: The topic to publish to.
        :param data: The dictionary to be published.
        """
        start_time = time.time()
        payload = json.dumps(data)
        msg_info = self.client.publish(topic, payload)
        # print(f"Published message {msg_info.mid} at time {time.time()}")
        # self.unacked_publish.add(msg_info.mid)

        # Wait for all message to be published
        # while len(self.unacked_publish):
        #     pass

        # msg_info.wait_for_publish()

    def publish_msg(self, topic, msg: MqttMessageBase = None):
        """
        Publish a message object to a specified topic.

        :param topic: The topic to publish to.
        :param msg: The message object to be published.
        """
        try:
            if topic not in self.topic_to_message_map:
                raise ValueError(f"Topic {topic} not found in topic_to_message_map")
            if not isinstance(msg, self.topic_to_message_map[topic]):
                raise ValueError(f"Message {msg} is not an instance of {self.topic_to_message_map[topic]}")
            payload = msg.convert_to_payload()
            msg_info = self.client.publish(topic, payload)
        except Exception as e:
            print(f"Error publishing message: {e}")

    def run(self, keep_alive: int = None):
        """Connect to the broker and start the MQTT loop."""
        self.connect(keep_alive)
        self.client.loop_start()

    def stop(self):
        """Disconnect from the broker and stop the MQTT loop."""
        self.client.disconnect()
        self.client.loop_stop()

class MQTTSubscriber(Thread):
    def __init__(self, broker_address="localhost", topic_to_message_map: dict[str, MqttMessageBase] = None):
        """
        Initialize the MQTT Subscriber with a broker address and a topic-to-message map.

        :param broker_address: Address of the MQTT broker.
        :param topic_to_message_map: Dictionary mapping topics to message classes.
        """
        super().__init__()
        self.broker_address = broker_address
        self.topic_to_message_map = topic_to_message_map
        self.client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2)
        self.client.on_connect = self.on_connect
        self.client.on_message = self.on_message
        self.client.on_subscribe = self.on_subscribe
        self.client.on_unsubscribe = self.on_unsubscribe
        self.latest_messages = {}

        self.last_timestamp = None

    def on_connect(self, client, userdata, flags, rc, properties):
        """
        Callback for when the client receives a CONNACK response from the server.

        :param client: The client instance for this callback.
        :param userdata: The private user data as set in Client() or userdata_set().
        :param flags: Response flags sent by the broker.
        :param rc: The connection result.
        :param properties: Properties associated with the connection (MQTTv5 only).
        """
        print(f"Connected to broker with result code {rc}")
        for topic in self.topic_to_message_map.keys():
            self.client.subscribe(topic)

    def on_message(self, client, userdata, message):
        """
        Callback for when a PUBLISH message is received from the server.

        :param client: The client instance for this callback.
        :param userdata: The private user data as set in Client() or userdata_set().
        :param message: An instance of MQTTMessage.
        """
        try:
            if message.topic in self.topic_to_message_map:
                message_class = self.topic_to_message_map[message.topic]
                if message_class is not None:
                    # Initialize the message class and convert payload
                    msg_instance = message_class()
                    msg_instance.convert_to_message(message.payload)

                    self.latest_messages[message.topic] = {'read': False, 'data': msg_instance}
                else:
                    # Load payload as a dictionary
                    data = json.loads(message.payload.decode())
                    self.latest_messages[message.topic] = {'read': False, 'data': data}
            else:
                print(f"Topic {message.topic} not found in topic_to_message_map")
        except json.JSONDecodeError as e:
            print(f"Error decoding JSON: {e}")
        except Exception as e:
            print(f"Error processing message: {e}")

    def get_latest_message(self, topic):
        """
        Retrieve the latest message for a given topic.

        :param topic: The topic to retrieve the message for.
        :return: The latest message data or None if not available.
        """
        if topic not in self.latest_messages:
            return None
        if self.latest_messages[topic]['read']:
            return None
        self.latest_messages[topic]['read'] = True
        return self.latest_messages[topic]['data']

    def on_subscribe(self, client, userdata, mid, reason_code_list, properties):
        """
        Callback for when the client subscribes to a topic.

        :param client: The client instance for this callback.
        :param userdata: The private user data as set in Client() or userdata_set().
        :param mid: Message ID.
        :param reason_code_list: List of reason codes for the subscription.
        :param properties: Properties associated with the subscription (MQTTv5 only).
        """
        print(f"Subscribed to topic: {client}")

    def on_unsubscribe(self, client, userdata, mid, reason_code_list, properties):
        """
        Callback for when the client unsubscribes from a topic.

        :param client: The client instance for this callback.
        :param userdata: The private user data as set in Client() or userdata_set().
        :param mid: Message ID.
        :param reason_code_list: List of reason codes for the unsubscription.
        :param properties: Properties associated with the unsubscription (MQTTv5 only).
        """
        print(f"Unsubscribed from topic: {client}")

    def connect(self):
        """Connect to the MQTT broker."""
        self.client.connect(self.broker_address)
        print(f"Connected to broker at {self.broker_address}")

    def run(self):
        """Connect to the broker and start the MQTT loop."""
        self.connect()
        self.client.loop_forever()

    def stop(self):
        """Disconnect from the broker and stop the MQTT loop."""
        self.client.disconnect()
        self.client.loop_stop()

if __name__ == "__main__":
    # mqttc = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2)
    # mqttc.enable_logger()

    # mqttc.connect("localhost")
    # mqttc.loop_start()
    # Load an image and convert it to a NumPy array
    image = cv2.imread('test/image.jpeg')
    _, buffer = cv2.imencode('.jpeg', image)
    image_as_text = base64.b64encode(buffer).decode('utf-8')

    subscriber = MQTTSubscriber(broker_address="localhost", topic_to_message_map={"/test": None, "/test2": None})
    subscriber.start()

    subscriber2 = MQTTSubscriber(broker_address="localhost", topic_to_message_map={"/test": None})
    subscriber2.start()

    time.sleep(1)

    publisher = MQTTPublisher(broker_address="localhost", topic_to_message_map={"/test": None})
    publisher.run()

    # Publish the image as a base64 string
    publisher.publish_dict("/test", {"image": image_as_text, "timestamp": time.time()})

    # publisher.publish("/test", {"test": time.time()})

    publisher2 = MQTTPublisher(broker_address="localhost", topic_to_message_map={"/test2": None})
    publisher2.run()

    publisher2.publish_dict("/test2", {"test": "test2"})

    while True:
        # keep going and interrupt with ctrl+c
        try:
            # publisher.publish_dict("/test", {"image": image_as_text, "timestamp": time.time()})
            # run at 300 hz
            time.sleep(1/300)
        except KeyboardInterrupt:
            break

    publisher.stop()
    publisher2.stop()
    subscriber.stop()
