import paho.mqtt.client as mqtt
import json
import threading

class RobotController:
    def __init__(self, velocity_broker="localhost", orientation_broker="localhost", velocity=0.01):
        self.state = {"position": 0, "velocity": 0, "angle": 0}
        self.executing = False
        self.velocity = velocity  # Default velocity
        self.desired_pos = 0
        self.stop_event = threading.Event()

        # Set up MQTT clients
        self.orientation_sub = mqtt.Client()
        self.orientation_sub.on_message = self.on_message
        self.orientation_sub.connect(orientation_broker)
        self.orientation_sub.subscribe("robot/orientation")

        self.velocity_pub = mqtt.Client()
        self.velocity_pub.connect(velocity_broker)

    def publish_velocity(self, linear, angular):
        data = {"linear": linear, "angular": angular}
        self.velocity_pub.publish("robot/velocity", json.dumps(data))
        print(f"Velocity published: {data}")

    def stop_velocity(self, delay):
        def stop():
            self.publish_velocity(0, 0)
            print("Velocity stopped.")
            self.executing = False
            self.stop_event.set()  # Signal that stopping is complete
        threading.Timer(delay, stop).start()

    def on_message(self, client, userdata, msg):
        payload = json.loads(msg.payload.decode())
        self.state["position"] = payload.get("position", 0)
        self.state["velocity"] = payload.get("velocity", 0)
        self.state["angle"] = payload.get("angle", 0)
        if not self.executing:
            print(f"State updated: {self.state}")

    def get_trajectory(self, desired_pos):
        self.executing = True
        command_time = abs(self.state["position"] - desired_pos) / self.velocity
        velocity_command = -self.velocity if self.state["position"] > desired_pos else self.velocity
        return {"velocity": velocity_command, "angular": 0, "time": command_time}

    def move_to_pos(self, desired_pos):
        traj = self.get_trajectory(desired_pos)
        print(traj)
        self.publish_velocity(traj["velocity"], traj["angular"])
        self.stop_velocity(traj["time"])
        self.stop_event.wait()  # Wait until stop_velocity completes

    def move(self, desired_pos):
        self.desired_pos = desired_pos
        self.executing = False
        try:
            self.orientation_sub.loop_start()
            self.move_to_pos(self.desired_pos)
        except KeyboardInterrupt:
            print("Operation stopped by user.")
        finally:
            print("Disconnecting Clients")
            self.orientation_sub.loop_stop()
            self.orientation_sub.disconnect()
            self.velocity_pub.disconnect()

if __name__ == "__main__":
    controller = RobotController()
    controller.move(desired_pos=0.1)