"use client";

import React, {
  useState,
  useCallback,
  useMemo,
  useRef,
  useEffect,
} from "react";
import { motion, Reorder } from "framer-motion";
import { CommandBlock } from "@/components/CommandBlock";
import { RepeatBlock } from "@/components/RepeatBlock";
import { TrashIcon } from "lucide-react";
import mqtt, { MqttClient } from "mqtt";

interface Command {
  id: string;
  type: "movement" | "rotation" | "sound" | "repeat" | "wait" | "if";
  command: string;
  steps?: number;
  sound?: string;
  children?: Command[];
  condition?: string;
}

export default function Home() {
  const [commands, setCommands] = useState<Command[]>([]);
  const mainRef = React.useRef<HTMLDivElement>(null);
  const [isDragging, setIsDragging] = useState(false);
  const [draggedId, setDraggedId] = useState<string | null>(null);
  const [isOverTrash, setIsOverTrash] = useState(false);
  const hasAddedCommand = React.useRef(false);
  const [connectionStatus, setConnectionStatus] = useState("Disconnected");
  const mqttClientRef = useRef<MqttClient | null>(null);
  const currentDragId = useRef<string | null>(null);
  const [isExecuting, setIsExecuting] = useState(false);

  useEffect(() => {
    const hostname = window.location.hostname;
    const username = hostname.split("-")[0];
    const mqttHost = username.includes(".")
      ? hostname
      : `${username}-bracketbot.local`;

    const client = mqtt.connect(`ws://${mqttHost}:9001`);
    mqttClientRef.current = client;

    client.on("connect", () => {
      setConnectionStatus("Connected");
      client.subscribe("/mapping/traversability_grid");
      client.subscribe("/mapping/robot_pose_grid_coords");
    });

    client.on("error", (error) => {
      setConnectionStatus(`Connection failed: ${error}`);
    });

    return () => {
      if (client) client.end();
    };
  }, []);

  const publishVelocity = (linear: number, angular: number) => {
    if (!mqttClientRef.current) return;
    const payload = JSON.stringify({
      timestamp: Date.now(),
      linear_velocity_mps: linear,
      angular_velocity_radps: angular,
    });
    mqttClientRef.current.publish("/control/target_velocity", payload);
  };

  const executeCommand = async (command: Command) => {
    return new Promise<void>((resolve) => {
      switch (command.type) {
        case "movement":
          if (command.command === "forward") {
            publishVelocity(0.5, 0);
            setTimeout(() => {
              publishVelocity(0, 0);
              resolve();
            }, command.steps! * 1000);
          } else if (command.command === "backward") {
            publishVelocity(-0.35, 0);
            setTimeout(() => {
              publishVelocity(0, 0);
              resolve();
            }, command.steps! * 1000);
          }
          break;

        case "rotation":
          const isLeft = command.command.includes("left");
          publishVelocity(0, isLeft ? 45.0 : -45.0);
          setTimeout(() => {
            publishVelocity(0, 0);
            resolve();
          }, command.steps! * 100);
          break;
        case "wait":
          setTimeout(resolve, command.steps! * 1000);
          break;

        case "sound":
          if (mqttClientRef.current) {
            mqttClientRef.current.publish(
              "`robot/speak`",
              command.sound || "meow"
            );
            setTimeout(resolve, 1000);
          } else {
            resolve();
          }
          break;

        case "repeat":
          executeCommands(command.children || [], command.steps || 1).then(
            resolve
          );
          break;

        case "if":
          if (command.children) {
            executeCommands(command.children).then(resolve);
          } else {
            resolve();
          }
          break;
      }
    });
  };

  const executeCommands = async (
    commandList: Command[],
    repeatCount: number = 1
  ) => {
    for (let i = 0; i < repeatCount; i++) {
      for (const command of commandList) {
        await executeCommand(command);
      }
    }
  };

  const handleDragEnd = useCallback((event: any, sourceId: string) => {
    const dragEventId = `${Date.now()}-${Math.random()}`;

    if (!mainRef.current) return;

    const mainRect = mainRef.current.getBoundingClientRect();
    const { clientX, clientY } = event;

    if (
      clientX < mainRect.left ||
      clientX > mainRect.right ||
      clientY < mainRect.top ||
      clientY > mainRect.bottom
    )
      return;

    // Find target repeat block
    const repeatBlocks = document.querySelectorAll(".repeat-block");
    let targetRepeatBlock: Element | null = null;

    repeatBlocks.forEach((block) => {
      const rect = block.getBoundingClientRect();
      if (
        clientX >= rect.left &&
        clientX <= rect.right &&
        clientY >= rect.top &&
        clientY <= rect.bottom
      ) {
        targetRepeatBlock = block;
      }
    });

    // Single state update
    setCommands((prev) => {
      if (hasAddedCommand.current) {
        return prev;
      }

      const type =
        sourceId === "repeat"
          ? ("repeat" as const)
          : sourceId.includes("left") || sourceId.includes("right")
          ? ("rotation" as const)
          : sourceId === "sound"
          ? ("sound" as const)
          : sourceId === "wait"
          ? ("wait" as const)
          : sourceId === "if"
          ? ("if" as const)
          : ("movement" as const);

      const newCommand: Command = {
        id: `${type}-${Date.now()}-${Math.random().toString(36).substr(2, 9)}`,
        type,
        command: sourceId,
        steps: type === "rotation" ? 15 : type === "repeat" ? 2 : 1,
        children: (type === "repeat" || type === "if") ? [] : undefined,
        condition: type === "if" ? "path_stopped" : undefined,
      };

      if (targetRepeatBlock) {
        const repeatBlockId = targetRepeatBlock.getAttribute("data-id");
        if (!repeatBlockId) return prev;

        const newCommands = [...prev];
        addCommandToRepeatBlock(newCommands, repeatBlockId, newCommand);
        hasAddedCommand.current = true; // Mark that we've added the command
        return newCommands;
      }

      hasAddedCommand.current = true; // Mark that we've added the command
      return [...prev, newCommand];
    });

    // Reset the ref after a short delay
    setTimeout(() => {
      hasAddedCommand.current = false;
    }, 100);
  }, []); // Add dependencies if needed

  const handleDelete = useCallback((commandId: string) => {
    setCommands((prev) => {
      const deleteCommandFromArray = (commands: Command[]): Command[] => {
        return commands.filter((cmd) => {
          if (cmd.id === commandId) {
            return false;
          }
          if (cmd.children) {
            cmd.children = deleteCommandFromArray(cmd.children);
          }
          return true;
        });
      };

      const newCommands = deleteCommandFromArray([...prev]);
      return newCommands;
    });
  }, []);

  const handleCanvasItemDragStart = useCallback((id: string) => {
    setDraggedId(id);
    currentDragId.current = id;
    setIsDragging(true);
  }, []);

  const handleCanvasItemDragEnd = useCallback(() => {
    setIsDragging(false);
  }, []);

  const findCommandById = (
    commands: Command[],
    id: string
  ): [Command | null, Command[] | null] => {
    for (let i = 0; i < commands.length; i++) {
      if (commands[i].id === id) {
        return [commands[i], commands];
      }
      if (commands[i].children) {
        const [found, parent] = findCommandById(commands[i].children!, id);
        if (found) {
          return [found, parent];
        }
      }
    }
    return [null, null];
  };

  // Memoize handlers
  const handleStepsChange = useCallback(
    (commandId: string, newSteps: number) => {
      setCommands((prev) => {
        const newCommands = [...prev];
        const [command, parent] = findCommandById(newCommands, commandId);
        if (command && parent) {
          const index = parent.indexOf(command);
          parent[index] = { ...command, steps: newSteps };
        }
        return newCommands;
      });
    },
    []
  );

  const handleSoundChange = useCallback(
    (commandId: string, newSound: string) => {
      setCommands((prev) =>
        prev.map((cmd) =>
          cmd.id === commandId ? { ...cmd, sound: newSound } : cmd
        )
      );
    },
    []
  );

  // Memoize renderCommands
  const renderCommands = useCallback(
    (commandsToRender: Command[]) => {
      return (
        <Reorder.Group
          axis="y"
          values={commandsToRender}
          onReorder={(newOrder) => {
            console.log("Canvas State:", {
              repeatBlocks: newOrder.filter((cmd) => cmd.type === "repeat"),
              allCommands: newOrder,
              totalCommands: newOrder.length,
              breakdown: {
                repeat: newOrder.filter((cmd) => cmd.type === "repeat").length,
                movement: newOrder.filter((cmd) => cmd.type === "movement")
                  .length,
                rotation: newOrder.filter((cmd) => cmd.type === "rotation")
                  .length,
                sound: newOrder.filter((cmd) => cmd.type === "sound").length,
              },
            });

            // Log nested commands in repeat blocks
            newOrder.forEach((cmd) => {
              if (cmd.type === "repeat") {
                console.log(`Repeat Block ${cmd.id}:`, {
                  steps: cmd.steps,
                  children: cmd.children,
                  childrenCount: cmd.children?.length || 0,
                });
              }
            });

            setCommands((prev) => {
              // If these are nested commands, we need to update the parent
              if (commandsToRender !== prev) {
                const parentCommand = prev.find(
                  (cmd) => cmd.children === commandsToRender
                );
                if (parentCommand) {
                  return prev.map((cmd) =>
                    cmd.id === parentCommand.id
                      ? { ...cmd, children: newOrder }
                      : cmd
                  );
                }
              }
              return newOrder;
            });
          }}
          className=""
        >
          {commandsToRender.map((command) => (
            <Reorder.Item
              key={command.id}
              value={command}
              className="command-item"
              drag
              dragListener={true}
              onDragStart={(event) => {
                event.stopPropagation();
                handleCanvasItemDragStart(command.id);
              }}
              onDragEnd={(event) => {
                event.stopPropagation();
                handleCanvasItemDragEnd();
              }}
            >
              <motion.div
                layout
                initial={{ opacity: 0, y: 20 }}
                animate={{ opacity: 1, y: 0 }}
                exit={{ opacity: 0, y: -20 }}
                transition={{ duration: 0.2 }}
              >
                {command.type === "repeat" || command.type === "if" ? (
                  <div className={`${command.type}-block`} data-id={command.id}>
                    {command.type === "repeat" ? (
                      <RepeatBlock
                        id={command.id}
                        steps={command.steps || 2}
                        onStepsChange={(steps) => handleStepsChange(command.id, steps)}
                        inCanvas={true}
                      >
                        {command.children && command.children.length > 0 && (
                          <div className="nested-commands">
                            {renderCommands(command.children)}
                          </div>
                        )}
                      </RepeatBlock>
                    ) : (
                      <CommandBlock
                        id={command.id}
                        type="if"
                        inCanvas={true}
                        condition={command.condition}
                        onConditionChange={(condition) => {
                          setCommands((prev) => {
                            const newCommands = [...prev];
                            const [cmd, parent] = findCommandById(newCommands, command.id);
                            if (cmd && parent) {
                              const index = parent.indexOf(cmd);
                              parent[index] = { ...cmd, condition };
                            }
                            return newCommands;
                          });
                        }}
                      >
                        {command.children && command.children.length > 0 && (
                          <div className="nested-commands pl-4 border-l-2 border-black/10">
                            {renderCommands(command.children)}
                          </div>
                        )}
                      </CommandBlock>
                    )}
                  </div>
                ) : (
                  <CommandBlock
                    id={command.id}
                    type={command.type}
                    inCanvas={true}
                    steps={command.steps}
                    sound={command.sound}
                    command={command.command}
                    onStepsChange={(steps) =>
                      handleStepsChange(command.id, steps)
                    }
                    onSoundChange={(sound) =>
                      handleSoundChange(command.id, sound)
                    }
                    condition={command.condition}
                  >
                    {command.type === "movement"
                      ? "Forward"
                      : command.type === "rotation"
                      ? `Turn ${
                          command.command.includes("left") ? "Left" : "Right"
                        }`
                      : "Play Sound"}
                  </CommandBlock>
                )}
              </motion.div>
            </Reorder.Item>
          ))}
        </Reorder.Group>
      );
    },
    [
      handleCanvasItemDragStart,
      handleCanvasItemDragEnd,
      handleStepsChange,
      handleSoundChange,
    ]
  );

  // Memoize main render
  const mainContent = useMemo(() => {
    return commands.length === 0 ? (
      <p className="text-gray-500">Drop commands here</p>
    ) : (
      renderCommands(commands)
    );
  }, [commands, renderCommands]);

  // Helper functions
  const removeCommandById = (commands: Command[], id: string): Command[] => {
    return commands.filter((cmd) => {
      if (cmd.id === id) return false;
      if (cmd.children) {
        cmd.children = removeCommandById(cmd.children, id);
      }
      return true;
    });
  };

  const addCommandToRepeatBlock = (
    commands: Command[],
    repeatBlockId: string,
    commandToAdd: Command
  ): boolean => {
    for (const cmd of commands) {
      if (cmd.id === repeatBlockId) {
        if (!cmd.children) cmd.children = [];

        // Check if command already exists in children
        const exists = cmd.children.some(
          (child) => child.id === commandToAdd.id
        );
        if (exists) {
          return true;
        }

        cmd.children.push(commandToAdd);
        return true;
      }
      if (
        cmd.children &&
        addCommandToRepeatBlock(cmd.children, repeatBlockId, commandToAdd)
      ) {
        return true;
      }
    }
    return false;
  };

  const isCommandInsideRepeat = (
    repeatCommand: Command | null,
    commandId: string | null
  ): boolean => {
    if (!repeatCommand || !commandId) return false;
    if (!repeatCommand.children) return false;

    return repeatCommand.children.some((child) => {
      if (child.id === commandId) return true;
      if (child.type === "repeat") {
        return isCommandInsideRepeat(child, commandId);
      }
      return false;
    });
  };

  // Create a unique ID generator function
  const generateUniqueId = (type: string) => {
    return `${type}-${Date.now()}-${Math.random().toString(36).substr(2, 9)}`;
  };

  return (
    <div className="flex h-screen relative">
      <aside className="w-72 border-r bg-gray-50 p-4 space-y-8">
        <div>
          <h1 className="my-6 font-medium">Drag and drop from below!</h1>
        </div>

        {/* Movement Commands */}
        <div>
          <h2 className="text-sm font-medium text-gray-600 mb-6">Movement</h2>
          <div className="space-y-6">
            <motion.div
              drag
              dragSnapToOrigin
              style={{ zIndex: 50 }}
              onDragEnd={(event) => {
                event.stopPropagation();
                handleDragEnd(event, "forward");
              }}
              dragTransition={{ bounceStiffness: 600, bounceDamping: 20 }}
            >
              <CommandBlock id="forward" type="movement" command="forward" steps={1}>
                Forward
              </CommandBlock>
            </motion.div>
            <motion.div
              drag
              dragSnapToOrigin
              style={{ zIndex: 50 }}
              onDragEnd={(event) => handleDragEnd(event, "backward")}
            >
              <CommandBlock id="backward" type="movement" command="backward" steps={1}>
                Move Backward
              </CommandBlock>
            </motion.div>
          </div>
        </div>

        {/* Rotation Commands */}
        <div>
          <h2 className="text-sm font-medium text-gray-600 mb-6">Rotation</h2>
          <div className="space-y-4">
            <motion.div
              drag
              dragSnapToOrigin
              style={{ zIndex: 50 }}
              onDragEnd={(event) => handleDragEnd(event, "left")}
            >
              <CommandBlock id="left" type="rotation" command="left" steps={15}>
                Turn Left
              </CommandBlock>
            </motion.div>
            <motion.div
              drag
              dragSnapToOrigin
              style={{ zIndex: 50 }}
              onDragEnd={(event) => handleDragEnd(event, "right")}
            >
              <CommandBlock id="right" type="rotation" command="right" steps={15}>
                Turn Right
              </CommandBlock>
            </motion.div>
          </div>
        </div>

        {/* Actions */}
        <div>
          <h2 className="text-sm font-medium text-gray-600 mb-6">Actions</h2>
          <div className="space-y-4">
            <motion.div
              drag
              dragSnapToOrigin
              style={{ zIndex: 50 }}
              onDragEnd={(event) => handleDragEnd(event, "sound")}
            >
              <CommandBlock id="sound" type="sound" sound="meow">
                Play Sound
              </CommandBlock>
            </motion.div>
            <motion.div
              drag
              dragSnapToOrigin
              style={{ zIndex: 50 }}
              onDragEnd={(event) => handleDragEnd(event, "wait")}
            >
              <CommandBlock id="wait" type="wait" steps={1}>
                Wait
              </CommandBlock>
            </motion.div>
          </div>
        </div>

        {/* Control */}
        <div>
          <h2 className="text-sm font-medium text-gray-600 mb-6">Control</h2>
          <div className="space-y-4">
            <motion.div
              drag
              dragSnapToOrigin
              style={{ zIndex: 50 }}
              onDragEnd={(event) => handleDragEnd(event, "repeat")}
            >
              <RepeatBlock id="repeat" steps={2} onStepsChange={() => {}} />
            </motion.div>
            
            <motion.div
              drag
              dragSnapToOrigin
              style={{ zIndex: 50 }}
              onDragEnd={(event) => handleDragEnd(event, "if")}
            >
              <CommandBlock 
                id="if" 
                type="if" 
                condition="path_stopped"
                command="if"
              >
                If
              </CommandBlock>
            </motion.div>
          </div>
        </div>
      </aside>

      <main ref={mainRef} className="flex-1 relative overflow-hidden">
        <div className="flex flex-col h-full">
          <div className="flex items-center justify-center p-4 border-b gap-4">
            <div
              className={`w-3 h-3 rounded-full ${
                connectionStatus === "Connected" ? "bg-green-500" : "bg-red-500"
              }`}
            />
            <span
              className={`text-sm ${
                connectionStatus === "Connected"
                  ? "text-green-600"
                  : "text-red-600"
              }`}
            >
              {connectionStatus}
            </span>

            <motion.button
              initial={false}
              animate={{
                scale: isExecuting ? 0.95 : 1,
                backgroundColor:
                  commands.length === 0 || connectionStatus === "Disconnected"
                    ? "rgb(156 163 175)" // gray-400
                    : isExecuting
                    ? "rgb(75 85 99)" // gray-600
                    : "rgb(34 197 94)", // green-500
              }}
              whileHover={
                commands.length === 0 || isExecuting
                  ? {}
                  : { backgroundColor: "rgb(22 163 74)" } // green-600
              }
              className={`
          flex px-6 py-2 rounded-lg font-medium items-center gap-2 text-white
          ${
            commands.length === 0
              ? "cursor-not-allowed"
              : isExecuting
              ? "cursor-wait"
              : "cursor-pointer"
          }
        `}
              disabled={
                commands.length === 0 ||
                isExecuting ||
                connectionStatus === "Disconnected"
              }
              onClick={async () => {
                try {
                  setIsExecuting(true);
                  console.log("Starting program execution...");
                  console.log("Current commands:", commands);
                  await executeCommands(commands);
                } finally {
                  setIsExecuting(false);
                }
              }}
            >
              <motion.div
                animate={{
                  rotate: isExecuting ? 360 : 0,
                }}
                transition={{
                  repeat: isExecuting ? Infinity : 0,
                  duration: 1,
                  ease: "linear",
                }}
              >
                {isExecuting ? (
                  <svg
                    className="h-5 w-5 text-white"
                    xmlns="http://www.w3.org/2000/svg"
                    fill="none"
                    viewBox="0 0 24 24"
                  >
                    <circle
                      className="opacity-25"
                      cx="12"
                      cy="12"
                      r="10"
                      stroke="currentColor"
                      strokeWidth="4"
                    />
                    <path
                      className="opacity-75"
                      fill="currentColor"
                      d="M4 12a8 8 0 018-8V0C5.373 0 0 5.373 0 12h4zm2 5.291A7.962 7.962 0 014 12H0c0 3.042 1.135 5.824 3 7.938l3-2.647z"
                    />
                  </svg>
                ) : (
                  <svg
                    className="h-5 w-5"
                    fill="none"
                    viewBox="0 0 24 24"
                    stroke="currentColor"
                  >
                    <path
                      strokeLinecap="round"
                      strokeLinejoin="round"
                      strokeWidth={2}
                      d="M14.752 11.168l-3.197-2.132A1 1 0 0010 9.87v4.263a1 1 0 001.555.832l3.197-2.132a1 1 0 000-1.664z"
                    />
                    <path
                      strokeLinecap="round"
                      strokeLinejoin="round"
                      strokeWidth={2}
                      d="M21 12a9 9 0 11-18 0 9 9 0 0118 0z"
                    />
                  </svg>
                )}
              </motion.div>
              <span>{isExecuting ? "Executing..." : "Run"}</span>
            </motion.button>
          </div>
          <div className="flex-1 p-4 border-2 border-dashed rounded-lg">
            {mainContent}
          </div>
        </div>
      </main>

      {/* Bottom-right controls */}
      <div className="absolute bottom-4 right-4 flex gap-4">
        <motion.div
          className={`p-4 rounded-lg transition-colors cursor-pointer
            border-2 ${
              isOverTrash && isDragging
                ? "border-red-600 bg-red-200"
                : isDragging
                ? "border-red-300 bg-red-100"
                : "border-gray-300 hover:border-red-300 hover:bg-red-50 bg-gray-100 group"
            }`}
          style={{ zIndex: 50 }}
          animate={{
            scale: isOverTrash && isDragging ? 1.3 : isDragging ? 1.2 : 1,
          }}
          onMouseEnter={() => isDragging && setIsOverTrash(true)}
          onMouseLeave={() => setIsOverTrash(false)}
          onMouseUp={() => {
            if (isOverTrash && draggedId) {
              handleDelete(draggedId);
              setDraggedId(null);
              currentDragId.current = null;
              setIsOverTrash(false);
            }
          }}
        >
          <TrashIcon
            className={`w-6 h-6 transition-colors ${
              isOverTrash && isDragging
                ? "text-red-600"
                : isDragging
                ? "text-red-500"
                : "text-gray-500 group-hover:text-red-500"
            }`}
          />
        </motion.div>
        <motion.button
          className={`px-6 py-2 rounded-lg font-medium 
            flex items-center gap-2 text-white
            ${
              commands.length === 0
                ? "cursor-not-allowed bg-gray-400"
                : "cursor-pointer bg-red-500 hover:bg-red-600"
            }`}
          initial={false}
          animate={{
            scale: 1,
            backgroundColor:
              commands.length === 0
                ? "rgb(156 163 175)" // gray-400
                : "rgb(239 68 68)", // red-500
          }}
          whileHover={
            commands.length === 0 ? {} : { backgroundColor: "rgb(220 38 38)" } // red-600
          }
          style={{ zIndex: 50 }}
          disabled={commands.length === 0}
          onClick={() => setCommands([])}
        >
          <svg
            className="h-5 w-5"
            fill="none"
            viewBox="0 0 24 24"
            stroke="currentColor"
          >
            <path
              strokeLinecap="round"
              strokeLinejoin="round"
              strokeWidth={2}
              d="M19 7l-.867 12.142A2 2 0 0116.138 21H7.862a2 2 0 01-1.995-1.858L5 7m5 4v6m4-6v6m1-10V4a1 1 0 00-1-1h-4a1 1 0 00-1 1v3M4 7h16"
            />
          </svg>
          <span>Clear All</span>
        </motion.button>
      </div>
    </div>
  );
}
