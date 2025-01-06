import { init, tx } from 'svelte-instantdb';

export const db = init({
  appId: 'f8690ab7-9a6e-4062-a809-1edbec33d960'
});

export { tx };