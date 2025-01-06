import importlib.util
import os
from datetime import datetime

import click
import toml

import utils.paths as PTH


@click.command()
@click.argument('toml_path',
                type=click.Path(exists=True),
                required=False,
                default=str(PTH.REPO_PATH / 'messages' / 'bot_quickstart_msgs.toml'))
def main(toml_path):
    """
    Main function to generate MQTT message classes from a TOML file.

    Args:
        toml_path (str): Path to the TOML file containing message definitions.
    """
    # Load the TOML file
    with open(toml_path, 'r') as file:
        messages = toml.load(file)

    # Directory to save generated message classes
    output_dir = PTH.REPO_PATH / 'utils' / 'messages'
    os.makedirs(output_dir, exist_ok=True)

    # Mapping of TOML types to Python types
    type_mapping = {
        'f32': 'np.float32',
        'd64': 'np.float64',
        'u8': 'np.uint8',
        'u32': 'np.uint32',
        'u64': 'np.uint64',
        'int': 'int',
        'float': 'float',
        'str': 'str',
        'bool': 'bool',
        'dict': 'dict',
        'list': 'list',
    }

    # Generate Python classes
    for message_name, fields in messages.items():
        class_name = message_name.upper() + '_MSG'
        class_file = os.path.join(output_dir, f"{class_name.lower()}.py")

        with open(class_file, 'w') as f:
            file_header_msg = f"Auto-generated MQTT message class from TOML configuration {toml_path} at {datetime.now()}\n\n"
            f.write("# " + file_header_msg)

            f.write("import json\n\n")
            f.write("from utils.messages.mqtt_message_base import MqttMessageBase\n\n\n")
            f.write(f"class {class_name}(MqttMessageBase):\n")
            
            # Define class members
            for field, value in fields.items():
                if isinstance(value, list) and len(value) == 2:
                    field_type, size = value
                    # Process fields with type and size
                    if not field.startswith('topic_'):
                        base_type = type_mapping.get(field_type, 'str')
                        if size == '1':
                            python_type = base_type
                        else:
                            if field_type in ['f32', 'd64', 'u8', 'u64']:
                                python_type = f'np.ndarray[{base_type}]'
                            else:
                                python_type = f'list[{base_type}]'
                        f.write(f"    {field}: {python_type} = None\n")
                else:
                    # Process fields with only a single value (e.g., topic fields)
                    if field.startswith('topic_'):
                        # Handle topic fields if necessary
                        pass

            # Define __init__ method
            f.write("\n    def __init__(self, ")
            f.write(", ".join([
                f"{field}: {type_mapping.get(value[0], 'str') if value[1] == '1' else ('np.ndarray[' + type_mapping.get(value[0], 'str') + ']' if value[0] in ['f32', 'd64', 'u8', 'u64'] else 'list[' + type_mapping.get(value[0], 'str') + ']')} = None"
                for field, value in fields.items() if not field.startswith('topic_')
            ]))
            f.write("):\n")
            f.write("        \"\"\"Initialize the message class with given fields.\"\"\"\n")
            for field, value in fields.items():
                if not field.startswith('topic_'):
                    if isinstance(value, list) and len(value) == 2:
                        f.write(f"        self.{field} = {field}\n")
            
            # Define convert_to_payload method
            f.write("\n    def convert_to_payload(self) -> str:\n")
            f.write("        \"\"\"Convert the message fields to a JSON payload.\"\"\"\n")
            f.write("        try:\n")
            f.write("            data = {\n")
            for field, value in fields.items():
                if not field.startswith('topic_'):
                    if isinstance(value, list) and len(value) == 2:
                        f.write(f"                '{field}': self.{field},\n")
            f.write("            }\n")
            f.write("            return json.dumps(data)\n")
            f.write("        except (TypeError, ValueError) as e:\n")
            f.write("            raise Exception(f'Error converting to payload: {e}')\n")

            # Define convert_to_message method
            f.write("\n    def convert_to_message(self, payload):\n")
            f.write("        \"\"\"Convert a JSON payload to message fields.\"\"\"\n")
            f.write("        try:\n")
            f.write("            data = json.loads(payload)\n")
            f.write("            if 'data' in data:\n")
            f.write("                data = data['data']\n")
            for field, value in fields.items():
                if not field.startswith('topic_'):
                    if isinstance(value, list) and len(value) == 2:
                        f.write(f"            self.{field} = data['{field}']\n")
            f.write("        except (json.JSONDecodeError, KeyError) as e:\n")
            f.write("            raise Exception(f'Error converting from payload: {e}')\n")

    # Update topic_to_message_type.py
    topic_to_message_map = {}
    for message_name, fields in messages.items():
        class_name = message_name.upper() + '_MSG'
        module_name = f"utils.messages.{class_name.lower()}"
        
        # Dynamically import the module
        spec = importlib.util.spec_from_file_location(module_name, os.path.join(output_dir, f"{class_name.lower()}.py"))
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        
        # Get the class from the module
        message_class = getattr(module, class_name)

        for field, value in fields.items():
            if isinstance(value, list) and len(value) == 2:
                continue
            if field.startswith('topic_'):
                topic_to_message_map[fields[field]] = message_class

    with open(PTH.REPO_PATH / 'utils' / 'topic_to_message_type.py', 'w') as f:
        file_header_msg = f"Auto-generated topic to message type mapping from TOML configuration {toml_path} at {datetime.now()}\n\n"
        f.write("# " + file_header_msg)

        # Track imported message classes
        imported_classes = set()

        # Add import statements for each message class
        for topic, message_class in topic_to_message_map.items():
            if message_class not in imported_classes:
                f.write(f"from utils.messages.{message_class.__name__.lower()} import {message_class.__name__}\n")
                imported_classes.add(message_class)

        # Define topic variables
        f.write("\n# Topic variables\n")
        for message_name, fields in messages.items():
            for field, value in fields.items():
                if field.startswith('topic_'):
                    f.write(f"{field.upper()} = '{value}'\n")

        f.write("\n# Topic to message type mapping\n")
        f.write("topic_to_message_type = {\n")
        for topic, message_class in topic_to_message_map.items():
            f.write(f"    '{topic}': {message_class.__name__},\n")
        f.write("}\n")

if __name__ == '__main__':
    main()
