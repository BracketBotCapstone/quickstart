# Robot Control Web Interface

A web-based control interface for robot operation built with Next.js and MQTT communication.

## Features

- Real-time robot control via web interface
- Touch-friendly mobile controls
- Keyboard control support (Arrow keys)
- Text-to-speech capability
- Connection status monitoring
- Responsive design for all devices

## Prerequisites

- Node.js 18.17 or later
- Yarn package manager
- Modern web browser with WebSocket support

## Getting Started

1. Clone the repository:

```bash

git clone <your-repository-url>

cd examples/noCode
```

2. Install dependencies:

```bash
yarn install
```

3. Make sure that your Bracket bot server is running first

```bash
python3 nodes/start_all_nodes.py
```

4. Start your development server


5. Open your browser and navigate to [http://localhost:3000](http://localhost:3000)

## Usage
Drag and drop and run it :)

### Connection Status
- Green indicator: Connected
- Red indicator: Disconnected
