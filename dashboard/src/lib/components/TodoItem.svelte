<script lang="ts">
  import type { Todo } from '../types/todo';
  import { todoStore } from '../stores/todoStore.svelte';

  const { todo } = $props<{ todo: Todo }>();

  function handleToggle() {
    todoStore.toggle(todo.id);
  }

  function handleDelete() {
    todoStore.delete(todo.id);
  }
</script>

<div class="todo-item">
  <input
    type="checkbox"
    checked={todo.completed}
    on:change={handleToggle}
  />
  <span class="todo-text" class:completed={todo.completed}>{todo.text}</span>
  <button on:click={handleDelete}>Delete</button>
</div>

<style>
  .todo-item {
    display: flex;
    align-items: center;
    gap: 1rem;
    padding: 0.5rem;
    border-bottom: 1px solid #ccc;
  }

  .todo-text {
    color: #333;
    flex: 1;
  }

  .completed {
    text-decoration: line-through;
    opacity: 0.6;
  }

  button {
    margin-left: auto;
    background-color: #ff3e00;
    color: white;
    border: none;
    padding: 0.5rem 1rem;
    border-radius: 4px;
    cursor: pointer;
  }

  button:hover {
    background-color: #ff2d00;
  }
</style>