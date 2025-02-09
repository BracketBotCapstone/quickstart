from anthropic import Anthropic
from dotenv import load_dotenv
from pydantic import BaseModel
from typing import Any, Dict
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

    def __repr__(self):
        return f"{self.intent}({''.join([f'{k}: {str(v)}' for (k, v) in self.params.items()])}): {self.description}"
    

class Action(BaseModel):
    code: str


class ActionBuilder:
    def __init__(self, *commands):
        self.commands = {c.intent: c for c in commands}
    
    @property
    def system_prompt(self):
        return {
            "role": "system",
            "content": f"""
            You are a robot action classifier. You will also be given a list of commands (atomic actions)
            which the robot can perform, and the parameters that action needs. You will then be given a string of text
            from a user, describing the action it wants the robot to take. Your job is to write python code
            which the robot should execute. Write in each function and its parameters as keyword arguments
            in order to execute the action the user describes. The functions do not return anything.
            
            DO NOT DO ANY OF THE FOLLOWING:
            - use or import libraries which are not builtin to base python (e.g., no NumPy or cv2)
            - put the code in a function or add "if __name__ == '__main__'"
            - add comments of any kind
            
            
            Do not return any explanation, just the program.

            List of possible commands and parameters:
            {" ".join([str(c) for c in self.commands.values()])}

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

        return resp
    
    
if __name__ == "__main__":
    rotation_degrees = ParamInput(
        return_type=float, 
        description="robot rotation amount in degrees, right is positive and left is negative"
    )
    drive_distance = ParamInput(
        return_type=float, 
        description="distance to go in metres"
    )
    rotate_degrees = CommandInput(
        intent="rotate", 
        description="rotate x degrees", 
        params={"x": rotation_degrees}
    )
    drive_forward = CommandInput(
        intent="drive_forward", 
        description="drive forward x metres", 
        params={"x": drive_distance}
    )

    resp = ActionBuilder(drive_forward, rotate_degrees).classify("drive forward 2 metres")
    print(resp)

    resp = ActionBuilder(drive_forward, rotate_degrees).classify("rotate the robot 60 degrees to the left")
    print(resp)

    resp = ActionBuilder(drive_forward, rotate_degrees).classify("drive forward 2 metres then rotate the robot 20 degrees to the right")
    print(resp)

    resp = ActionBuilder(drive_forward, rotate_degrees).classify("trace out a square with sides that are 1 metre")
    print(resp)

    resp = ActionBuilder(drive_forward, rotate_degrees).classify("drive 2 metres. flip a coin. if heads, go left 2 metres. else, go right 2 metres.")
    print(resp)

    def rotate(x):
        print(f"the robot is rotating {x} degrees")

    def drive_forward(x):
        print(f"the robot is driving {x} metres")

    exec(resp.code)
