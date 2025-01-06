<script lang="ts">
import { onMount } from "svelte";
import TodoInput from "../lib/components/TodoInput.svelte";
import TodoItem from "../lib/components/TodoItem.svelte";
import { db } from "../lib/db/config";
import type { Todo } from "../lib/types/todo";
import RobotControl from "../lib/components/RobotControl.svelte";
import OccupancyGrid from "../lib/components/OccupancyGrid.svelte";
import { mqttService } from "../lib/services/mqtt";
import Cursors from "../lib/components/Cursors.svelte";
import { createRoom, type Room } from "../lib/services/room";
const query = { todos: {} };
const { current } = db.useQuery(query);

let todos = $state<Todo[]>([]);
let connectionStatus = $state("Disconnected");
const roomInitialized = $state(false);
const room = $state<Room | null>(null);

$effect(() => {
	todos =
		current.data?.todos?.map(
			(todo) =>
				({
					id: todo.id,
					text: todo.text || "",
					completed: !!todo.completed,
					createdAt: todo.createdAt || Date.now(),
				}) as Todo,
		) || [];
});

onMount(async () => {
	// roomInitialized = true;
	// room = await createRoom('chat', 'dashboard');
	// console.log('room',room);
	mqttService.onStatusChange((status) => {
		connectionStatus = status;
	});
});

const userCursorColor = "tomato";
</script>

    <div class="min-h-screen bg-[#f7f9fc]">
      <!-- <Cursors {room} {userCursorColor}> -->
        <header class="sticky top-0 z-50 bg-white border-b border-[#e0e4e8] shadow-sm">
          <div class="max-w-7xl mx-auto px-8 py-4 flex justify-between items-center">
            <div class="logo">
              <h1 class="text-[#111827] text-2xl font-semibold">BracketBot</h1>
            </div>
            <div class="flex items-center gap-4">
              <a 
                href="https://bracket.bot/updates" 
                target="_blank"
                rel="noopener noreferrer"
                class="flex items-center gap-2 text-gray-600 hover:text-gray-900 transition-colors"
              >
                <span class="text-sm font-medium">Updates</span>
                <svg class="w-4 h-4" fill="none" stroke="currentColor" viewBox="0 0 24 24">
                  <path stroke-linecap="round" stroke-linejoin="round" stroke-width="2" d="M10 6H6a2 2 0 00-2 2v10a2 2 0 002 2h10a2 2 0 002-2v-4M14 4h6m0 0v6m0-6L10 14" />
                </svg>
              </a>
              <div class="flex items-center gap-2 px-4 py-2 rounded-full {connectionStatus === 'Connected' ? 'bg-green-50 border border-green-200' : 'bg-red-50 border border-red-200'}">
                <span class="w-2 h-2 rounded-full {connectionStatus === 'Connected' ? 'bg-green-500' : 'bg-red-500'}"></span>
                <span class="text-sm font-medium text-[#111827]">{connectionStatus}</span>
              </div>
            </div>
          </div>
        </header>

        <main class="max-w-7xl mx-auto px-8 py-8 space-y-8">
          <div class="bg-white p-6 rounded-xl shadow-sm border border-[#e5e7eb]">
            <h2 class="text-xl font-semibold text-[#111827] text-center mb-6">Navigation Map</h2>
            <OccupancyGrid />
          </div>

          <div class="max-w-4xl mx-auto bg-white p-6 rounded-xl shadow-sm border border-[#e5e7eb]">
            <h2 class="text-xl font-semibold text-[#111827] text-center mb-6">Robot Control</h2>
            <RobotControl />
          </div>

          <div class="max-w-2xl mx-auto bg-white p-6 rounded-xl shadow-sm border border-[#e5e7eb]">
            <h2 class="text-xl font-semibold text-[#111827] text-center mb-6">Bracket Bot Updates</h2>
            <TodoInput />
            
            {#if current.isLoading}
              <p class="text-center text-gray-500">Loading todos...</p>
            {:else if current.error}
              <p class="text-center text-red-500">Error: {current.error.message}</p>
            {:else}
              <div class="space-y-2">
                {#each todos as todo (todo.id)}
                  <TodoItem {todo} />
                {/each}
              </div>
            {/if}
          </div>
        </main>
      <!-- </Cursors> -->
    </div>
