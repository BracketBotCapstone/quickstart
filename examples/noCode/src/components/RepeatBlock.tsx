import React from 'react';
import { motion } from 'framer-motion';
import { Input } from "@/components/ui/input";

interface RepeatBlockProps {
  id: string;
  steps: number;
  onStepsChange: (steps: number) => void;
  children?: React.ReactNode;
  inCanvas?: boolean;
}

export function RepeatBlock({ 
  id, 
  steps = 1, 
  onStepsChange,
  children,
  inCanvas = false 
}: RepeatBlockProps) {
  return (
    <div className="relative" data-id={id}>
      <motion.div
        layout
        className={`
          relative p-3 mb-2 rounded-lg border-2
          bg-teal-100 border-teal-300
          cursor-move transition-all
          ${inCanvas ? 'hover:scale-[1.02]' : ''}
        `}
      >
        <div className="flex items-center gap-2">
          <svg className="w-4 h-4" fill="none" viewBox="0 0 24 24" stroke="currentColor">
            <path strokeLinecap="round" strokeLinejoin="round" strokeWidth={2} d="M4 4v5h.582m15.356 2A8.001 8.001 0 004.582 9m0 0H9m11 11v-5h-.581m0 0a8.003 8.003 0 01-15.357-2m15.357 2H15" />
          </svg>
          <span>Repeat</span>
          <Input
            type="number"
            value={steps}
            onChange={(e) => onStepsChange(Math.max(1, parseInt(e.target.value) || 1))}
            className="w-16 h-8 text-center"
            min={1}
            max={100}
          />
          <span>times</span>
        </div>
        {children && (
          <motion.div layout className="mt-2 pl-4 border-l-2 border-teal-300">
            {children}
          </motion.div>
        )}
      </motion.div>
    </div>
  );
} 