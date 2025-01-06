<script lang="ts">
import { onMount } from "svelte";
import { mqttService } from "../services/mqtt";
import type { GridData } from "../types/grid";
import type { RobotPoseData } from "../types/robotpose";
import { gridSettings } from "../stores/gridSettings";

const width = 800;
const gridSize = 400;
const cellSize = width / gridSize;
let canvas: HTMLCanvasElement;

let showHeatmap = $state(false);

$effect(() => {
	showHeatmap = $gridSettings.showHeatmap;
});

// Define color stops for gradient
const COLORS = {
	stops: [
		{ time: 0, color: [0, 255, 0] }, // Green at 0s
		{ time: 30, color: [255, 255, 0] }, // Yellow at 30s
		{ time: 60, color: [255, 165, 0] }, // Orange at 60s
		{ time: 90, color: [255, 0, 0] }, // Red at 90s
	],
};

function interpolateColor(ageSeconds: number): string {
	// Find the two color stops we're between
	let start = COLORS.stops[0];
	let end = COLORS.stops[COLORS.stops.length - 1];

	for (let i = 0; i < COLORS.stops.length - 1; i++) {
		if (
			ageSeconds >= COLORS.stops[i].time &&
			ageSeconds < COLORS.stops[i + 1].time
		) {
			start = COLORS.stops[i];
			end = COLORS.stops[i + 1];
			break;
		}
	}

	// Calculate how far we are between the two stops (0-1)
	const fraction = (ageSeconds - start.time) / (end.time - start.time);

	// Interpolate between colors
	const r = Math.round(
		start.color[0] + (end.color[0] - start.color[0]) * fraction,
	);
	const g = Math.round(
		start.color[1] + (end.color[1] - start.color[1]) * fraction,
	);
	const b = Math.round(
		start.color[2] + (end.color[2] - start.color[2]) * fraction,
	);

	return `rgb(${r}, ${g}, ${b})`;
}

// Define types first
type ClickLog = {
	x: number;
	y: number;
	timestamp: string;
};

// State variables for click tracking
const myClicks = $state<ClickLog[]>([]);
const otherTargets = $state<ClickLog[]>([]);
const MAX_LOG_ENTRIES = 5;

type GridCell = {
	occupied: boolean;
	timestamp: number;
	lastUpdate: number;
};

// Create a 2D array to store grid state
const gridState = $state<GridCell[][]>(
	Array(gridSize)
		.fill(null)
		.map(() =>
			Array(gridSize)
				.fill(null)
				.map(() => ({
					occupied: false,
					timestamp: 0,
					lastUpdate: 0,
				})),
		),
);

// Add window view state
const windowState = $state<{
	lastUpdate: number;
	changes: Set<string>; // Track changed cells by "x,y" key
}>({
	lastUpdate: Date.now(),
	changes: new Set(),
});

function updateGrid(ctx: CanvasRenderingContext2D, gridData: GridData) {
	const now = Date.now();
	windowState.changes.clear();

	for (let y = 0; y < gridData.width; y++) {
		for (let x = 0; x < gridData.width; x++) {
			const idx = y * gridData.width + x;
			const newState = gridData.flattened_grid_list[idx];
			const cell = gridState[y as number][x as number];

			// Only update if state changed
			console.log("updating grid");
			if (cell.occupied !== newState) {
				cell.occupied = newState;
				cell.timestamp = gridData.timestamp * 1000;
				cell.lastUpdate = now;
				windowState.changes.add(`${x},${y}`);

				// Draw just this cell
				drawCell(ctx, x, gridSize - 1 - y, cell);
			} else {
				console.log("not draw");
			}
		}
	}

	windowState.lastUpdate = now;
}

function drawCell(
	ctx: CanvasRenderingContext2D,
	x: number,
	y: number,
	cell: GridCell,
) {
	if (cell.occupied) {
		const color = calculateColor(cell.timestamp);
		ctx.fillStyle = color;
	} else {
		ctx.fillStyle = "black";
	}
	console.log("drawing cell");
	ctx.fillRect(x * cellSize + 1, y * cellSize + 1, cellSize, cellSize);
}

// Add a state variable to store the robot pose
let robotPose: RobotPoseData | null = null;

onMount(() => {
	if (!canvas) return;
	const ctx = canvas.getContext("2d");
	if (!ctx) return;

	ctx.globalAlpha = 1;
	ctx.globalCompositeOperation = "source-over";

	drawEmptyGrid(ctx);
	mqttService.onGridUpdate((gridData: GridData) => {
		updateGrid(ctx, gridData);
	});

	// Update the robot pose whenever new data is received
	mqttService.onRobotPoseUpdate((poseData: RobotPoseData) => {
		robotPose = poseData;
	});

	// Optional: Periodic redraw for smooth decay visualization
	const redrawInterval = setInterval(() => {
		console.log("redrawing");
		if (showHeatmap) {
			drawEmptyGrid(ctx);
			// Redraw all occupied cells
			gridState.forEach((row, y) => {
				row.forEach((cell, x) => {
					if (cell.occupied) {
						const color = calculateColor(cell.timestamp);
						ctx.fillStyle = color;
						ctx.fillRect(
							x * cellSize + 1,
							y * cellSize + 1,
							cellSize,
							cellSize,
						);
					}
				});
			});
			// Draw the robot pose last if available
			if (robotPose) {
				displayRobotPose(ctx, robotPose);
			}
		}
	}, 1000); // Update every second

	return () => clearInterval(redrawInterval);
});

function drawEmptyGrid(ctx: CanvasRenderingContext2D) {
	// Fill background black
	ctx.fillStyle = "black";
	ctx.fillRect(0, 0, width, width);

	// Draw grid lines in dark gray
	ctx.strokeStyle = "#333";
	ctx.lineWidth = 0.5;

	for (let i = 0; i <= gridSize; i++) {
		const pos = i * cellSize;
		ctx.beginPath();
		ctx.moveTo(pos, 0);
		ctx.lineTo(pos, width);
		ctx.stroke();

		ctx.beginPath();
		ctx.moveTo(0, pos);
		ctx.lineTo(width, pos);
		ctx.stroke();
	}
}

function calculateColor(timestamp: number): string {
	if (!showHeatmap) {
		return "rgb(255, 255, 255)"; // Solid white if no heatmap
	}

	const ageMs = Date.now() - timestamp;
	const ageSeconds = ageMs / 1000;

	return interpolateColor(Math.min(ageSeconds, 90)); // Cap at 90s
}

function handleClick(event: MouseEvent) {
	if (!canvas) return;
	const rect = canvas.getBoundingClientRect();
	const scaleX = canvas.width / rect.width;
	const scaleY = canvas.height / rect.height;

	const x = Math.floor(((event.clientX - rect.left) * scaleX) / cellSize);
	const y = Math.floor(((event.clientY - rect.top) * scaleY) / cellSize);

	// Add to myClicks
	const timestamp = new Date().toLocaleTimeString();
	myClicks.unshift({ x, y, timestamp });
	if (myClicks.length > MAX_LOG_ENTRIES) {
		myClicks.pop();
	}

	// Publish to target point
	mqttService.publish("/planning/target_point", {
		timestamp: 0.0,
		x_grid: x,
		y_grid: y,
	});
	console.log("mqttService", mqttService);
	console.log("Click coordinates:", { x, y });
	console.log("Click timestamp:", timestamp);
	console.log("Published target point:", JSON.stringify([x, y]));
}

function displayRobotPose(ctx: CanvasRenderingContext2D, poseData: RobotPoseData) {
	const { timestamp,x_grid, y_grid, theta_rad } = poseData;
	// Draw the robot position on the grid with a larger size
	const robotSize = cellSize * 10; // Increase the size by 50%
	ctx.fillStyle = "blue"; // Use a distinct color for the robot position
	ctx.fillRect(
		(400-x_grid) * cellSize + 1 - (robotSize - cellSize) / 2,
		y_grid * cellSize + 1 - (robotSize - cellSize) / 2,
		robotSize,
		robotSize
	);

	// Calculate the center of the robot
	const centerX = (400-x_grid) * cellSize + cellSize / 2;
	const centerY = y_grid * cellSize + cellSize / 2;

	// Calculate the endpoint of the arrow
	const arrowLength = cellSize * 10; // Length of the arrow
	const endX = centerX - arrowLength * Math.cos(theta_rad - Math.PI / 2);
	const endY = centerY - arrowLength * Math.sin(theta_rad - Math.PI / 2);

	// Draw the arrow
	ctx.strokeStyle = "red"; // Use a distinct color for the arrow
	ctx.lineWidth = 2;
	ctx.beginPath();
	ctx.moveTo(centerX, centerY);
	ctx.lineTo(endX, endY);
	ctx.stroke();
}
</script>

<div class="grid-container">
  <div class="controls">
    <label class="control">
      <input type="checkbox" bind:checked={showHeatmap}>
      Color Map by Age
    </label>
    {#if showHeatmap}
      <div class="time-brackets">
        <div class="bracket" style:background="rgb(0, 255, 0)">&lt; 30s</div>
        <div class="bracket" style:background="rgb(255, 255, 0)">30s-1m</div>
        <div class="bracket" style:background="rgb(255, 165, 0)">1m-1.5m</div>
        <div class="bracket" style:background="rgb(255, 0, 0)">&gt; 1.5m</div>
      </div>
    {/if}
  </div>

  <canvas 
    bind:this={canvas} 
    {width}
    height={width}
    class="canvas"
    on:click={handleClick}
  ></canvas>

  <div class="logs-container">
    <div class="log-panel">
      <h3>Your Clicks</h3>
      <div class="log-entries">
        {#each myClicks as click}
          <div class="log-entry">
            [{click.timestamp}] x: {click.x}, y: {click.y}
          </div>
        {/each}
        {#if myClicks.length === 0}
          <div class="empty-log">No clicks yet</div>
        {/if}
      </div>
    </div>

    <div class="log-panel">
      <h3>Other Users' Targets</h3>
      <div class="log-entries">
        {#each otherTargets as point}
          <div class="log-entry">
            [{point.timestamp}] x: {point.x}, y: {point.y}
          </div>
        {/each}
        {#if otherTargets.length === 0}
          <div class="empty-log">No other targets</div>
        {/if}
      </div>
    </div>
  </div>
</div>

<style>
  .grid-container {
    position: relative;
    width: 100%;
    max-width: 800px;
    margin: 0 auto;
    background: black;
    border-radius: 8px;
    overflow: hidden;
    box-shadow: 0 1px 3px rgba(0, 0, 0, 0.1);
    border: 1px solid #e5e7eb;
  }

  .canvas {
    display: block;
    width: 100%;
    height: 100%;
    background: black;
    image-rendering: pixelated;
    opacity: 1;
    mix-blend-mode: normal;
  }

  .click-log {
    position: absolute;
    bottom: 1rem;
    right: 1rem;
    background: rgba(255, 255, 255, 0.9);
    backdrop-filter: blur(4px);
    border-radius: 8px;
    box-shadow: 0 2px 4px rgba(0, 0, 0, 0.1);
    border: 1px solid #e5e7eb;
    padding: 1rem;
    max-width: 20rem;
  }

  .click-log h3 {
    font-size: 0.875rem;
    font-weight: 500;
    color: #374151;
    margin: 0 0 0.5rem 0;
  }

  .log-entries {
    display: flex;
    flex-direction: column;
    gap: 0.25rem;
  }

  .log-entry {
    font-family: monospace;
    font-size: 0.75rem;
    color: #4b5563;
  }

  .empty-log {
    font-size: 0.75rem;
    color: #6b7280;
    font-style: italic;
  }

  .controls {
    position: absolute;
    top: 1rem;
    right: 1rem;
    display: flex;
    gap: 1rem;
    z-index: 10;
  }

  .control {
    display: flex;
    align-items: center;
    gap: 0.5rem;
    background: rgba(255, 255, 255, 0.9);
    padding: 0.5rem;
    border-radius: 0.5rem;
    font-size: 0.875rem;
    color: #374151;
    cursor: pointer;
  }

  .control input {
    cursor: pointer;
  }

  .time-brackets {
    display: flex;
    gap: 0.5rem;
    margin-top: 0.5rem;
  }

  .bracket {
    padding: 0.25rem 0.5rem;
    border-radius: 0.25rem;
    font-size: 0.75rem;
    color: black;
    font-weight: 500;
  }

  .logs-container {
    position: absolute;
    bottom: 1rem;
    right: 1rem;
    display: flex;
    gap: 1rem;
  }

  .log-panel {
    background: rgba(255, 255, 255, 0.9);
    backdrop-filter: blur(4px);
    border-radius: 8px;
    box-shadow: 0 2px 4px rgba(0, 0, 0, 0.1);
    border: 1px solid #e5e7eb;
    padding: 1rem;
    max-width: 20rem;
  }
</style> 