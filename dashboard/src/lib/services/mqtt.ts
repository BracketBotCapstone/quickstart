import type { MqttClient, IClientOptions } from "mqtt";
import mqtt from "mqtt";
import type { GridData } from "../types/grid";
import type { TargetPointData } from "../types/targetpoint";
import type { RobotPoseData } from "../types/robotpose";
// const brokerUrl = "ws://10.147.17.131:9001";
const brokerUrl = "ws://10.0.0.50:9001";
// window.location.protocol === "https:"
// 	? ""
// 	: "ws://shrek-desktop.local:9001";

console.log("protocol: brokerURl", brokerUrl);
type CursorData = {
	x: number;
	y: number;
	color: string;
	timestamp: number;
};

type PresenceData = {
	name: string;
	color: string;
	timestamp: number;
};

type MQTTMessage =
	| string
	| number
	| boolean
	| Record<string, unknown>
	| Array<string | number>;

class MQTTService {
	private client: MqttClient | null = null;
	private statusCallback: ((status: string) => void) | null = null;
	private gridCallback: ((grid: GridData) => void) | null = null;
	private reconnectAttempts = 0;
	private maxReconnectAttempts = 5;
	private reconnectDelay = 2000;
	private cursorCallback: ((room: string, cursors: CursorData) => void) | null =
		null;
	private presenceCallback:
		| ((room: string, presence: PresenceData) => void)
		| null = null;
	private targetPointCallback:
		| ((targetPoint: TargetPointData, source: string) => void)
		| null = null;
	private robotPoseCallback:
		| ((robotPose: RobotPoseData, source: string) => void)
		| null = null;
	private clientId = `client_${Math.random().toString(16).slice(2, 8)}`;

	get id(): string {
		return this.clientId;
	}

	connect(url: string = brokerUrl) {
		const options: IClientOptions = {
			keepalive: 30,
			reconnectPeriod: this.reconnectDelay,
			connectTimeout: 5000,
			username: "lime",
			password: "1234",
			clean: true,
		};

		this.client = mqtt.connect(url, options);
		console.log("Attempting to connect to MQTT broker at", url);
		this.updateStatus("Connecting...");

		this.client.on("connect", () => {
			console.log("Connected to MQTT broker");
			this.reconnectAttempts = 0;
			this.updateStatus("Connected");
			this.client?.subscribe("/mapping/traversability_grid", { qos: 1 });
			this.client?.subscribe("/planning/target_point", { qos: 1 });
			this.client?.subscribe("presence/+", { qos: 1 });
			this.client?.subscribe("cursors/+", { qos: 1 });
			this.client?.subscribe("/mapping/robot_pose_grid_coords", { qos: 1 });
		});

		this.client.on("reconnect", () => {
			this.reconnectAttempts++;
			console.log(`Reconnection attempt ${this.reconnectAttempts}`);
			this.updateStatus(
				`Reconnecting... (${this.reconnectAttempts}/${this.maxReconnectAttempts})`,
			);

			if (this.reconnectAttempts >= this.maxReconnectAttempts) {
				console.error("Max reconnection attempts reached");
				this.client?.end();
				this.updateStatus("Connection failed - please refresh");
			}
		});

		this.client.on("offline", () => {
			console.log("MQTT client went offline");
			this.updateStatus("Disconnected - attempting to reconnect...");
		});

		this.client.on("error", (error) => {
			console.error("MQTT error:", error);
			this.updateStatus(`Connection error: ${error.message}`);
		});

		this.client.on("message", (topic: string, message: Buffer) => {
			console.log("Received message on topic:", topic);
			if (topic === "/mapping/traversability_grid") {
				try {
					const gridData: GridData = JSON.parse(message.toString());
					// console.log("Raw grid data:", message.toString());
					// console.log("Parsed grid data:", gridData);
					// console.log("Grid dimensions:", gridData.width);
					// console.log(
					// 	"True values:",
					// 	gridData.flattened_grid_list.filter((x) => x).length,
					// );

					this.gridCallback?.(gridData);
				} catch (e) {
					console.error("Failed to parse grid data:", e);
				}
			}
			if (topic === "/mapping/robot_pose_grid_coords") {
				try {
					const robotPose: RobotPoseData = JSON.parse(message.toString());
					// console.log("Raw robot pose data:", message.toString());
					// console.log("Parsed robot pose data:", robotPose);
					// console.log("X:", robotPose.x_grid);
					// console.log("Y:", robotPose.y_grid);
					this.robotPoseCallback?.(robotPose, this.clientId);
				} catch (e) {
					console.error("Failed to parse robot pose data:", e);
				}
			}
			if (topic.startsWith("presence/")) {
				const room = topic.split("/")[1];
				try {
					const presence = JSON.parse(message.toString());
					this.presenceCallback?.(room, presence);
				} catch (e) {
					console.error("Failed to parse presence data:", e);
				}
			}
			if (topic === "/planning/target_point") {
				try {
					const targetPoint: TargetPointData = JSON.parse(message.toString());
					// console.log("Raw target point data:", message.toString());
					// console.log("Parsed target point data:", targetPoint);
					// console.log("Point X:", targetPoint.x_grid);
					// console.log("Point Y:", targetPoint.y_grid);
					this.targetPointCallback?.(targetPoint, this.clientId);
				} catch (e) {
					console.error("Failed to parse target point:", e);
				}
			}
		});
	}

	disconnect() {
		if (this.client) {
			this.client.end();
			this.client = null;
			this.updateStatus("Disconnected");
		}
	}

	sendVelocity(linear: number, angular: number) {
		if (!this.client) return;

		this.publish("/control/target_velocity", {
			timestamp: 0.0,
			linear_velocity_mps: linear,
			angular_velocity_radps: angular,
		});
	}

	sendSpeech(text: string) {
		if (!this.client) return;
		this.client.publish("robot/speak", text);
	}

	onStatusChange(callback: (status: string) => void) {
		this.statusCallback = callback;
	}

	onGridUpdate(callback: (grid: GridData) => void) {
		this.gridCallback = callback;
	}

	onRobotPoseUpdate(callback: (robotPose: RobotPoseData) => void) {
		this.robotPoseCallback = callback;
	}

	private updateStatus(status: string) {
		this.statusCallback?.(status);
	}

	sendGridClick(x: number, y: number) {
		if (!this.client) return;

		const payload = JSON.stringify({ x, y });
		this.client.publish("grid/click", payload);
	}

	sendCursorPosition(
		room: string,
		position: { x: number; y: number; color: string },
	) {
		if (!this.client) return;

		const payload = JSON.stringify({
			...position,
			timestamp: Date.now(),
		});
		this.client.publish(`cursors/${room}`, payload);
	}

	onCursorUpdate(callback: (room: string, cursors: CursorData) => void) {
		this.cursorCallback = callback;
	}

	sendPresence(room: string, presence: { name: string; color: string }) {
		if (!this.client) return;

		const payload = JSON.stringify({
			...presence,
			timestamp: Date.now(),
		});
		this.client.publish(`presence/${room}`, payload);
	}

	onPresenceUpdate(callback: (room: string, presence: PresenceData) => void) {
		this.presenceCallback = callback;
	}

	publish(topic: string, message: MQTTMessage) {
		const payload = JSON.stringify({
			data: message,
			source: this.clientId,
			timestamp: Date.now(),
		});
		this.client?.publish(topic, payload);
	}

	onTargetPoint(callback: (targetPoint: TargetPointData, source: string) => void) {
		this.targetPointCallback = callback;
	}
}

export const mqttService = new MQTTService();
