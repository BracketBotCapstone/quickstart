import { init_experimental, tx } from 'svelte-instantdb';
import type { User } from '@instantdb/core';
import { i } from "@instantdb/core";
// Define types for InstantDB
export interface Message {
  text: string;
  timestamp: number;
}

const schema = i.graph(
  {
    "$users": i.entity({
      "email": i.any().unique().indexed(),
    }),
    "messages": i.entity({
      "createdAt": i.any(),
      "text": i.any(),
    }),
  },
  {
    "messages$users": {
      "forward": {
        "on": "messages",
        "has": "one",
        "label": "$users"
      },
      "reverse": {
        "on": "$users",
        "has": "many",
        "label": "messages"
      }
    }
  }
);

// Define message transaction types
type TxMessageSchema = {
  text?: string;
  createdAt?: number;
};

interface TxMessage {
  id: string;
  data: TxMessageSchema;
}

type RoomSchema = {
  chat: {
    presence: { name: string; color: string };
    topics: {
      emoji: {
        rotationAngle: number;
        directionAngle: number;
      };
    };
  };
};

const db = init_experimental<typeof schema>({
  appId: 'f8690ab7-9a6e-4062-a809-1edbec33d960',
  schema: schema.withRoomSchema<RoomSchema>(),
});

export interface AuthState {
  isLoading: boolean;
  error?: Error;
  user: User | null;
}

export interface PresenceData {
  name: string;
}

// Auth methods
export const auth = {
  useAuth: () => db.useAuth() as unknown as AuthState,
  sendMagicCode: (email: string) => db.auth.sendMagicCode({ email }),
  signInWithMagicCode: (email: string, code: string) => 
    db.auth.signInWithMagicCode({ email, code }),
  signOut: () => db.auth.signOut()
};

// Room methods with proper typing
export const createRoom = (roomName: keyof RoomSchema, roomId: string) => {
  // Use proper type for room
  const room = db.room(roomName, roomId) as ReturnType<typeof db.room>;
  return {
    room,
    usePresence: (initial = { peers: [], user: false }) => room.usePresence(initial),
    useTypingIndicator: (scope: string) => room.useTypingIndicator(scope)
  };
};

// Query methods with proper typing
export const getMessages = () => db.useQuery({ messages: {} });

// Transaction methods with proper typing
export const transactMessage = (transaction: TxMessage) => {
  const {  id: docId, data } = transaction;
  console.log('transactMessage', docId, data);
  return db.transact(tx.messages[docId].update(data));
};
export { tx, id } from 'svelte-instantdb';

export default db; 