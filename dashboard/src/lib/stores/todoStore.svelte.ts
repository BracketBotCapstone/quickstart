import { db, tx } from '../db/config';
import type { Todo } from '../types/todo';
import { id } from '@instantdb/core';

let todos = $state<Todo[]>([]);
let isLoading = $state(false);
let error = $state<Error | null>(null);

export const todoStore = {
  get todos() { return todos; },
  get isLoading() { return isLoading; },
  get error() { return error; },

  async load() {
    try {
      isLoading = true;
      const { current } = db.useQuery({
        todos: {
          $: {
            where: {},
            limit: 50,
          }
        }
      });
      
      todos = current.data?.todos.map(todo => ({
        id: todo.id,
        text: todo.text,
        completed: todo.completed,
        createdAt: todo.createdAt
      })) || [];
    } catch (e) {
      error = e instanceof Error ? e : new Error('Failed to load todos');
    } finally {
      isLoading = false;
    }
  },

  async add(text: string) {
    const todoId = id();
    await db.transact([
      tx.todos[todoId].update({
        text,
        completed: false,
        createdAt: Date.now()
      })
    ]);
    await this.load();
  },

  async toggle(todoId: string) {
    const todo = todos.find(t => t.id === todoId);
    if (todo) {
      await db.transact([
        tx.todos[todoId].update({
          completed: !todo.completed
        })
      ]);
      await this.load();
    }
  },

  async delete(todoId: string) {
    await db.transact([
      tx.todos[todoId].delete()
    ]);
    await this.load();
  }
};