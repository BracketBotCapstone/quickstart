<script lang="ts">
import { onMount, onDestroy } from "svelte";
import { mqttService } from "../services/mqtt";

let speechText = "";
let isConnected = $state(false);

onMount(() => {
	mqttService.onStatusChange((newStatus: string) => {
		isConnected = newStatus === "Connected";
	});
	mqttService.connect();
});

onDestroy(() => {
	mqttService.disconnect();
});

function handleStart(linear: number, angular: number) {
	if (!isConnected) return;
	mqttService.sendVelocity(linear, angular);
}

function handleEnd() {
	if (!isConnected) return;
	mqttService.sendVelocity(0, 0);
}

function sendSpeech() {
	if (!isConnected) return;
	mqttService.sendSpeech(speechText);
	speechText = "";
}

const buttonClass =
	"w-16 h-16 bg-gray-100 rounded-lg border border-gray-200 hover:bg-gray-200 active:bg-gray-300 touch-manipulation select-none transition-colors text-2xl font-bold text-gray-700 flex items-center justify-center";
</script>

<div class="flex flex-col items-center gap-6 max-w-sm mx-auto">
  <div class="grid grid-cols-3 gap-2 {!isConnected && 'opacity-50 pointer-events-none'}">
    <button 
      class={buttonClass}
      on:mousedown={() => handleStart(0.5, -1.0)}
      on:mouseup={handleEnd}
      on:mouseleave={handleEnd}
      on:touchstart|preventDefault={() => handleStart(0.5, -1.0)}
      on:touchend|preventDefault={handleEnd}
    >↖</button>
    <button 
      class={buttonClass}
      on:mousedown={() => handleStart(0.5, 0)}
      on:mouseup={handleEnd}
      on:mouseleave={handleEnd}
      on:touchstart|preventDefault={() => handleStart(0.5, 0)}
      on:touchend|preventDefault={handleEnd}
    >↑</button>
    <button 
      class={buttonClass}
      on:mousedown={() => handleStart(0.5, 1.0)}
      on:mouseup={handleEnd}
      on:mouseleave={handleEnd}
      on:touchstart|preventDefault={() => handleStart(0.5, 1.0)}
      on:touchend|preventDefault={handleEnd}
    >↗</button>
    <button 
      class={buttonClass}
      on:mousedown={() => handleStart(0, -1.0)}
      on:mouseup={handleEnd}
      on:mouseleave={handleEnd}
      on:touchstart|preventDefault={() => handleStart(0, -1.0)}
      on:touchend|preventDefault={handleEnd}
    >←</button>
    <button 
      class={buttonClass}
      on:mousedown={() => handleStart(0, 0)}
      on:mouseup={handleEnd}
      on:mouseleave={handleEnd}
      on:touchstart|preventDefault={() => handleStart(0, 0)}
      on:touchend|preventDefault={handleEnd}
    >●</button>
    <button 
      class={buttonClass}
      on:mousedown={() => handleStart(0, 1.0)}
      on:mouseup={handleEnd}
      on:mouseleave={handleEnd}
      on:touchstart|preventDefault={() => handleStart(0, 1.0)}
      on:touchend|preventDefault={handleEnd}
    >→</button>
    <button 
      class={buttonClass}
      on:mousedown={() => handleStart(-0.5, -1.0)}
      on:mouseup={handleEnd}
      on:mouseleave={handleEnd}
      on:touchstart|preventDefault={() => handleStart(-0.5, -1.0)}
      on:touchend|preventDefault={handleEnd}
    >↙</button>
    <button 
      class={buttonClass}
      on:mousedown={() => handleStart(-0.5, 0)}
      on:mouseup={handleEnd}
      on:mouseleave={handleEnd}
      on:touchstart|preventDefault={() => handleStart(-0.5, 0)}
      on:touchend|preventDefault={handleEnd}
    >↓</button>
    <button 
      class={buttonClass}
      on:mousedown={() => handleStart(-0.5, 1.0)}
      on:mouseup={handleEnd}
      on:mouseleave={handleEnd}
      on:touchstart|preventDefault={() => handleStart(-0.5, 1.0)}
      on:touchend|preventDefault={handleEnd}
    >↘</button>
  </div>

  <div class="flex gap-2 w-full max-w-xs">
    <input 
      type="text" 
      bind:value={speechText} 
      placeholder="Enter text to speak"
      disabled={!isConnected}
      class="flex-1 px-3 py-2 rounded-lg border border-gray-200 text-sm disabled:opacity-50 disabled:cursor-not-allowed"
    >
    <button 
      on:click={sendSpeech} 
      disabled={!isConnected || !speechText}
      class="px-4 py-2 bg-blue-500 text-white rounded-lg text-sm font-medium hover:bg-blue-600 disabled:opacity-50 disabled:cursor-not-allowed transition-colors"
    >
      Speak
    </button>
  </div>
</div> 
