import { writable } from "svelte/store";

export const gridSettings = writable({
	showHeatmap: true,
	showDecay: true,
});
