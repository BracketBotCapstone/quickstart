import React from 'react';
import { useDroppable } from '@dnd-kit/core';

interface Command {
  id: string;
  type: string;
}

interface CanvasProps {
  commands: Command[];
}

export function Canvas({ commands }: CanvasProps) {
  const {setNodeRef, isOver} = useDroppable({
    id: 'canvas',
  });

  return (
    <div
      ref={setNodeRef}
      className={`min-h-[300px] rounded-lg p-4 ${
        isOver ? 'bg-gray-200' : 'bg-gray-100'
      }`}
    >
      {commands.map((command) => (
        <div 
          key={command.id}
          className="bg-white border-2 border-blue-500 rounded-lg p-4 mb-2"
        >
          {command.type}
        </div>
      ))}
    </div>
  );
}

interface DroppableProps {
  children: React.ReactNode;
}

export function Droppable({ children }: DroppableProps) {
  const {isOver, setNodeRef} = useDroppable({
    id: 'droppable',
  });
  
  return (
    <div 
      ref={setNodeRef} 
      className={`
        relative w-full h-full transition-colors duration-200
        ${isOver ? 'bg-blue-50' : ''}
      `}
    >
      {children}
    </div>
  );
} 