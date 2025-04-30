'use client'

import React, { useEffect, useRef, useState } from "react";
import mqtt, { MqttClient } from "mqtt";

export default function Home() {
  const [connectionStatus, setConnectionStatus] = useState("Disconnected");
  const [speechText, setSpeechText] = useState("");
  const mqttClientRef = useRef<MqttClient | null>(null);

  const publishVelocity = (linear: number, angular: number) => {
    if (!mqttClientRef.current) return;
    const payload = JSON.stringify({
      timestamp: Date.now(),
      linear_velocity_mps: linear,
      angular_velocity_radps: angular,
    });
    mqttClientRef.current.publish("/control/target_velocity", payload);
  };

  const sendVelocity = (linear: number, angular: number) => {
    publishVelocity(linear, angular);
  };

  const sendSpeech = () => {
    if (!mqttClientRef.current) return;
    mqttClientRef.current.publish("`robot/speak`", speechText);
    setSpeechText("");
  };
  
  const handleKeyDown = (event: KeyboardEvent) => {
    const key = event.key.toLowerCase();
    switch (key) {
      case "w":
        publishVelocity(0.5, 0.0);
        break;
      case "s":
        publishVelocity(-0.5, 0.0);
        break;
      case "a":
        publishVelocity(0.0, 45.0);
        break;
      case "d":
        publishVelocity(0.0, -45.0);
        break;
      case "q":
        publishVelocity(0.0, 0.0);
        break;
      default:
        break;
    }
  };

  const handleKeyUp = (event:KeyboardEvent) => {
    const key = event.key.toLowerCase();
    if (["w", "a", "s", "d", "arrowup", "arrowdown", "arrowleft", "arrowright"].includes(key)) {
      publishVelocity(0.0, 0.0);
    }
  };

  useEffect(() => {
    
    const hostname = window.location.hostname;
    const username = hostname.split("-")[0];
    const mqttHost = username.includes(".") ? hostname : `${username}-bracketbot.local`;

    // Correctly formatted WebSocket URL
    const client = mqtt.connect(`ws://${mqttHost}:9001`);
    mqttClientRef.current = client;

    client.on("connect", () => {
      setConnectionStatus("Connected");
      // Subscribe to mapping topics if needed
      client.subscribe('/mapping/traversability_grid');
      client.subscribe('/mapping/robot_pose_grid_coords');
    });

    client.on("error", (error) => {
      setConnectionStatus(`Connection failed: ${error}`);
    });

    window.addEventListener("keydown", handleKeyDown);
    window.addEventListener("keyup", handleKeyUp);

    return () => {
      window.removeEventListener("keydown", handleKeyDown);
      window.removeEventListener("keyup", handleKeyUp);
      if (client) client.end();
    };
  }, []);

  return (
    <div className="min-h-screen p-8 pb-20 flex flex-col items-center justify-center gap-8 font-sans">
      <div className="w-full max-w-2xl text-center">
        <h2 className="text-2xl font-bold mb-6">Robot Controller</h2>
        <div
          className={`text-lg font-medium mb-8 ${
            connectionStatus.startsWith("Connected") ? "text-green-600" : "text-red-600"
          }`}
        >
          {connectionStatus}
        </div>

        {/* Control pad – values are adjusted to match the HTML example for at least the forward button */}
        <div className="grid grid-cols-3 gap-4 max-w-[280px] mx-auto mb-8">
          <button
            className="w-20 h-20 bg-gray-100 rounded-lg text-2xl flex items-center justify-center hover:bg-gray-200 active:bg-gray-300 transition-colors touch-manipulation select-none"
            onTouchStart={() => sendVelocity(0.35, -1.0)}
            onTouchEnd={() => sendVelocity(0, 0)}
          >
            ↖
          </button>
          <button
            className="w-20 h-20 bg-gray-100 rounded-lg text-2xl flex items-center justify-center hover:bg-gray-200 active:bg-gray-300 transition-colors touch-manipulation select-none"
            onTouchStart={() => sendVelocity(0.5, 0)}
            onTouchEnd={() => sendVelocity(0, 0)}
          >
            ↑
          </button>
          <button
            className="w-20 h-20 bg-gray-100 rounded-lg text-2xl flex items-center justify-center hover:bg-gray-200 active:bg-gray-300 transition-colors touch-manipulation select-none"
            onTouchStart={() => sendVelocity(0.35, 1.0)}
            onTouchEnd={() => sendVelocity(0, 0)}
          >
            ↗
          </button>
          <button
            className="w-20 h-20 bg-gray-100 rounded-lg text-2xl flex items-center justify-center hover:bg-gray-200 active:bg-gray-300 transition-colors touch-manipulation select-none"
            onTouchStart={() => sendVelocity(0, -1.0)}
            onTouchEnd={() => sendVelocity(0, 0)}
          >
            ←
          </button>
          <button
            className="w-20 h-20 bg-gray-100 rounded-lg text-2xl flex items-center justify-center hover:bg-gray-200 active:bg-gray-300 transition-colors touch-manipulation select-none"
            onTouchStart={() => sendVelocity(0, 0)}
            onTouchEnd={() => sendVelocity(0, 0)}
          >
            Stop
          </button>
          <button
            className="w-20 h-20 bg-gray-100 rounded-lg text-2xl flex items-center justify-center hover:bg-gray-200 active:bg-gray-300 transition-colors touch-manipulation select-none"
            onTouchStart={() => sendVelocity(0, 1.0)}
            onTouchEnd={() => sendVelocity(0, 0)}
          >
            →
          </button>
          <button
            className="w-20 h-20 bg-gray-100 rounded-lg text-2xl flex items-center justify-center hover:bg-gray-200 active:bg-gray-300 transition-colors touch-manipulation select-none"
            onTouchStart={() => sendVelocity(-0.35, -1.0)}
            onTouchEnd={() => sendVelocity(0, 0)}
          >
            ↙
          </button>
          <button
            className="w-20 h-20 bg-gray-100 rounded-lg text-2xl flex items-center justify-center hover:bg-gray-200 active:bg-gray-300 transition-colors touch-manipulation select-none"
            onTouchStart={() => sendVelocity(-0.35, 0)}
            onTouchEnd={() => sendVelocity(0, 0)}
          >
            ↓
          </button>
          <button
            className="w-20 h-20 bg-gray-100 rounded-lg text-2xl flex items-center justify-center hover:bg-gray-200 active:bg-gray-300 transition-colors touch-manipulation select-none"
            onTouchStart={() => sendVelocity(-0.35, 1.0)}
            onTouchEnd={() => sendVelocity(0, 0)}
          >
            ↘
          </button>
        </div>

        <div className="flex gap-4 justify-center items-center">
          <input
            type="text"
            id="speechText"
            placeholder="Enter text to speak"
            value={speechText}
            onChange={(e) => setSpeechText(e.target.value)}
            className="px-4 py-2 border rounded-lg text-base w-64 focus:outline-none focus:ring-2 focus:ring-blue-500"
          />
          <button
            onClick={sendSpeech}
            className="px-6 py-2 bg-blue-500 text-white rounded-lg hover:bg-blue-600 transition-colors"
          >
            Speak
          </button>
        </div>
      </div>
    </div>
  );
}