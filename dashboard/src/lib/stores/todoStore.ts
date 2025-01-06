import { db } from '../db/config';
import type { Todo } from '../types/todo';
import { createTodoActions } from '../actions/todoActions';

export function createTodoStore() {
  let todos = $state<Todo[]>([]);
  const actions = createTodoActions(todos);

  return {
    get todos() { return todos; },
    set todos(value: Todo[]) { todos = value; },
    ...actions
  };
}

export const todoStore = createTodoStore();