import { db } from "../db/config";

export type Room = {
  id: string;
  type: string;
  presence: {
    name: string;
    color: string;
  };
};

export const createRoom = (type: string, id: string): Room => {
  return {
    id,
    type,
    presence: {
      name: `user-${Math.random().toString(36).slice(2, 7)}`,
      color: `hsl(${Math.random() * 360}, 70%, 40%)`
    }
  };
};
