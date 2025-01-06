<script lang="ts">
import type { Room } from "../services/room";
import CustomCursor from "./CustomCursor.svelte";
import { Cursors } from "svelte-instantdb";

const { room, userCursorColor } = $props<{
	room: Room;
	userCursorColor: string;
}>();
</script>

<Cursors {room} {userCursorColor}>
  Move your cursor around! ✨
  {#snippet cursor({ color, presence })}
    <CustomCursor {color} name={presence?.name ?? ''} />
  {/snippet}
</Cursors>
