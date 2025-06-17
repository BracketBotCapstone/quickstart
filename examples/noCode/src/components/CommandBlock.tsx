import React from "react";
import { Input } from "@/components/ui/input";
import {
  Select,
  SelectContent,
  SelectItem,
  SelectTrigger,
  SelectValue,
} from "@/components/ui/select";
import { RotateCcw, RotateCw } from "lucide-react";

export type CommandType = "movement" | "rotation" | "sound" | 'wait' | "if";

interface CommandBlockProps {
  id: string;
  type: CommandType;
  children?: React.ReactNode;
  inCanvas?: boolean;
  onDragEnd?: () => void;
  steps?: number;
  onStepsChange?: (steps: number) => void;
  command?: string;
  condition?: string;
  onConditionChange?: (condition: string) => void;
  [key: string]: any;
}

const getBackgroundColor = (type: CommandType) => {
  switch (type) {
    case "movement":
      return "bg-amber-100 before:bg-amber-100 after:bg-amber-100";
    case "rotation":
      return "bg-pink-200 before:bg-pink-200 after:bg-pink-200";
    case "sound":
      return "bg-purple-100 before:bg-purple-100 after:bg-purple-100";
    case "wait":
      return "bg-red-200 before:bg-red-200 after:bg-red-200";
    case "if":
      return "bg-orange-100 before:bg-orange-100 after:bg-orange-100";
    default:
      return "bg-gray-100 before:bg-gray-100 after:bg-gray-100";
  }
};

const MoveCommand = ({
  direction,
  steps,
  onStepsChange,
}: {
  direction: "forward" | "backward";
  steps: number;
  onStepsChange: (steps: number) => void;
}) => (
  <div className="flex items-center gap-2">
    <span className="text-sm font-semibold">
      Move {direction === "forward" ? "Front" : "Back"}
    </span>
    <Input
      type="number"
      value={steps === 0 ? "" : steps}
      onChange={(e) => {
        const value = e.target.value;
        if (value === "") {
          onStepsChange(0);
          return;
        }
        const num = parseInt(value);
        if (!isNaN(num)) {
          onStepsChange(num >= 0 ? num : 0);
        }
      }}
      className="w-16 h-8 text-center bg-white/50 border-none focus:ring-2 focus:ring-amber-200 [appearance:textfield] [&::-webkit-outer-spin-button]:appearance-none [&::-webkit-inner-spin-button]:appearance-none"
      min={0}
      max={100}
      placeholder=""
    />
    <span className="text-sm font-semibold">steps</span>
  </div>
);

const TurnCommand = ({
  direction,
  steps,
  onStepsChange,
}: {
  direction: "left" | "right";
  steps: number;
  onStepsChange: (steps: number) => void;
}) => (
  <div className="flex items-center gap-2">
    <span className="text-sm font-semibold">Rotate</span>
    {direction === "left" ? (
      <RotateCcw className="w-4 h-4" />
    ) : (
      <RotateCw className="w-4 h-4" />
    )}
    <Input
      type="number"
      value={steps === 0 ? "" : steps}
      onChange={(e) => {
        const value = e.target.value;
        if (value === "") {
          onStepsChange(0);
          return;
        }
        const num = parseInt(value);
        if (!isNaN(num)) {
          onStepsChange(Math.max(1, num));
        }
      }}
      className="w-16 h-8 text-center bg-white/50 border-none focus:ring-2 focus:ring-pink-200 [appearance:textfield] [&::-webkit-outer-spin-button]:appearance-none [&::-webkit-inner-spin-button]:appearance-none"
      min={1}
      max={360}
      placeholder=""
    />
    <span className="text-sm font-semibold">degrees</span>
  </div>
);

const SoundCommand = ({
  sound,
  onSoundChange,
}: {
  sound: string;
  onSoundChange: (sound: string) => void;
}) => (
  <div className="flex items-center gap-2">
    <span className="text-sm font-semibold">Play</span>
    <Select value={sound} onValueChange={onSoundChange}>
      <SelectTrigger className="w-[140px] bg-white/50 border-none focus:ring-2 focus:ring-purple-200">
        <SelectValue placeholder="Select a sound" />
      </SelectTrigger>
      <SelectContent>
        <SelectItem value="meow">Meow</SelectItem>
        <SelectItem value="bruh">Bruh</SelectItem>
        <SelectItem value="anotherone">AnotherOne</SelectItem>
        <SelectItem value="rockLMFAO">ijustwannarock</SelectItem>
      </SelectContent>
    </Select>
  </div>
);

const WaitCommand = ({
  steps,
  onStepsChange,
}: {
  steps: number;
  onStepsChange: (steps: number) => void;
}) => (
  <div className="flex items-center gap-2">
    <span className="text-sm font-semibold">Wait</span>
    <Input
      type="number"
      value={steps === 0 ? "" : steps}
      onChange={(e) => {
        const value = e.target.value;
        if (value === "") {
          onStepsChange(0);
          return;
        }
        const num = parseInt(value);
        if (!isNaN(num)) {
          onStepsChange(Math.max(1, num));
        }
      }}
      className="w-16 h-8 text-center bg-white/50 border-none focus:ring-2 focus:ring-red-200 [appearance:textfield] [&::-webkit-outer-spin-button]:appearance-none [&::-webkit-inner-spin-button]:appearance-none"
      min={1}
      max={60}
      placeholder=""
    />
    <span className="text-sm font-semibold">seconds</span>
  </div>
);

const IfCommand = ({
  condition,
  onConditionChange,
}: {
  condition: string;
  onConditionChange: (condition: string) => void;
}) => (
  <div className="flex items-center gap-2">
    <span className="text-sm font-semibold">If</span>
    <Select value={condition} onValueChange={onConditionChange}>
      <SelectTrigger className="w-[140px] bg-white/50 border-none focus:ring-2 focus:ring-orange-200">
        <SelectValue placeholder="Select condition" />
      </SelectTrigger>
      <SelectContent>
        <SelectItem value="path_stopped">Path Stopped</SelectItem>
        <SelectItem value="path_clear">Path Clear</SelectItem>
        <SelectItem value="obstacle_detected">Obstacle Detected</SelectItem>
        <SelectItem value="battery_low">Battery Low</SelectItem>
      </SelectContent>
    </Select>
  </div>
);

export function CommandBlock({
  id,
  type,
  children,
  inCanvas = false,
  onDragEnd,
  steps = 1,
  onStepsChange = () => {},
  sound = "meow",
  onSoundChange = () => {},
  command,
  condition,
  onConditionChange,
  ...props
}: CommandBlockProps & {
  sound?: string;
  onSoundChange?: (sound: string) => void;
}) {
  const renderContent = () => {
    if (type === "movement") {
      const direction = command?.includes("backward") ? "backward" : "forward";
      return (
        <MoveCommand
          direction={direction}
          steps={steps}
          onStepsChange={onStepsChange}
        />
      );
    }
    if (type === "rotation") {
      const direction = command?.includes("left") ? "left" : "right";
      return (
        <TurnCommand
          direction={direction}
          steps={steps}
          onStepsChange={onStepsChange}
        />
      );
    }
    if (type === "sound") {
      return <SoundCommand sound={sound} onSoundChange={onSoundChange} />;
    }
    if (type === "wait") {
      return <WaitCommand steps={steps} onStepsChange={onStepsChange} />;
    }
    if (type === "if") {
      return (
        <IfCommand
          condition={condition || "path_stopped"}
          onConditionChange={onConditionChange || (() => {})}
        />
      );
    }
    return children;
  };

  return (
    <div
      {...props}
      className={`
        relative p-3
        ${getBackgroundColor(type)}
        border-2 border-black/10 
        rounded-lg
        shadow-sm
        transition-all
        hover:shadow-xl hover:shadow-black/5
        ${inCanvas ? "hover:scale-[1.01]" : ""}
        
        /* Top puzzle piece connector */
        before:content-['']
        before:absolute
        before:w-8
        before:h-4
        before:left-8
        before:top-0
        before:translate-y-[-85%]
        before:rounded-t-lg
        before:border-t-2
        before:border-l-2
        before:border-r-2
        before:border-black/10
      `}
    >
      <div className="flex items-center">{renderContent()}</div>
    </div>
  );
}
