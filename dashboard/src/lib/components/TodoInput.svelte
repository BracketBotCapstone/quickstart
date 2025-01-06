<script lang="ts">
  import { todoStore } from '../stores/todoStore.svelte';

  let newTodo = $state('');

  async function handleSubmit() {
    if (newTodo.trim()) {
      await todoStore.add(newTodo);
      newTodo = '';
    }
  }
</script>

<form on:submit|preventDefault={handleSubmit} class="todo-input">
  <input
    type="text"
    bind:value={newTodo}
    placeholder="What did you do this week?"
  />
  <button type="submit">Add</button>
</form>

<style>
  .todo-input {
    display: flex;
    gap: 1rem;
    margin-bottom: 1rem;
  }

  input {
    flex: 1;
    padding: 0.5rem;
    border: 1px solid #ccc;
    border-radius: 4px;
  }

  button {
    background-color: #4caf50;
    color: white;
  }

  button:hover {
    background-color: #45a049;
  }
</style>