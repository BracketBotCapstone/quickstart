import { db } from '../db/config';
import type { Todo } from '../types/todo';

export function createTodoActions(todos: Todo[]) {
  return {
    load: async () => {
      const result = await db.query({ todos: { where: {} } });
      todos = result.todos || [];
    },

    add: async (text: string) => {
      const newTodo: Omit<Todo, 'id'> = {
        text,
        completed: false,
        createdAt: Date.now(),
      };
      const result = await db.insert('todos', newTodo);
      todos = [...todos, { ...newTodo, id: result.id }];
    },

    toggle: async (id: string) => {
      const todo = todos.find(t => t.id === id);
      if (todo) {
        const updatedTodo = { ...todo, completed: !todo.completed };
        await db.update('todos', id, { completed: updatedTodo.completed });
        todos = todos.map(t => t.id === id ? updatedTodo : t);
      }
    },

    delete: async (id: string) => {
      await db.delete('todos', id);
      todos = todos.filter(t => t.id !== id);
    }
  };
}