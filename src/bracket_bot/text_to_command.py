from anthropic import Anthropic
from dotenv import load_dotenv
from pydantic import BaseModel, create_model
from typing import Any, Dict, List

from enum import Enum

import instructor

import os


load_dotenv()
        

class ParamInput(BaseModel):
    return_type: Any
    description: str
    
    def __repr__(self):
        return f"Type: {self.return_type}, Description: {self.description}"
        

class CommandInput(BaseModel):
    intent: str
    description: str
    params: Dict[str, ParamInput]
    fn: Any

    def __repr__(self):
        return f"{self.intent}({''.join([f'{k}: {str(v)}' for (k, v) in self.params.items()])}): {self.description}"


class ParamOutput(BaseModel):
    value: Any


class CommandOutput(BaseModel):
    intent: str
    params: Dict[str, ParamOutput]
    fn: Any = None

    def __call__(self):
        return self.fn(**{n: p.value for (n, p) in self.params.items()})
    

class Action(BaseModel):
    commands: List[CommandOutput]

    def __call__(self):
        for cmd in self.commands:
            cmd()


class ActionBuilder:
    def __init__(self, *commands):
        self.commands = {c.intent: c for c in commands}
    
    @property
    def system_prompt(self):
        return {
            "role": "system",
            "content": f"""\
            You are a robot action classifier. You will also be given a list of commands (atomic actions)
            which the robot can perform, and the parameters that action needs. You will then be given a string of text
            from a user, describing the action it wants the robot to take. Your job is to return a list of 
            commands which the robot should execute, and their parameters (populated in the value field of the Param object) 
            in order to execute the action the user describes. Do not return any explanation, just the list of commands.

            List of possible commands and parameters:
            {"\n".join([str(c) for c in self.commands.values()])}

            User requested action:
            """,
        }

    @property
    def classifier_model(self):
        return instructor.from_anthropic(client=Anthropic(
            api_key=os.environ.get("ANTHROPIC_API_KEY")
        ))
    
    def classify(self, text: str):
        resp, _ = self.classifier_model.chat.completions.create_with_completion(
            max_tokens=1024,
            messages=[
                self.system_prompt,
                {
                    "role": "user",
                    "content": text,
                }
            ],
            model="claude-3-5-sonnet-latest",
            response_model=Action
        )

        for cmd in resp.commands:
            cmd.fn = self.commands[cmd.intent].fn

        return resp
    
    
if __name__ == "__main__":
    rotation_degrees = ParamInput(
        return_type=float, 
        description="robot rotation amount in degrees"
    )
    drive_distance = ParamInput(
        return_type=float, 
        description="distance to go in metres"
    )
    drive_forward = CommandInput(
        intent="drive_forward", 
        description="drive forward x metres", 
        params={"x": drive_distance}, 
        fn=lambda x: print(x)
    )

    resp = ActionBuilder(drive_forward).classify("drive forward 2 metres")

    print(resp())
