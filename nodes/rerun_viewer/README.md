# To view the rerun data

If running the code launches as a node, you can view the rerun data on your local machine by setting your local IP in the [bot_quickstart.toml](../../config/bot_quickstart.toml) file.

> **Note:** After you set the IP above, ensure to run `CONFIG=bot_quickstart CONFIG_MSGS=bot_quickstart_msgs make` to update the configs

Then on your local machine:

```bash
pip3 install rerun-sdk
# In a new terminal
rerun
```

An example if not using the node and running a python script directly on the Rpi:
On the Rpi:
```bash
python3
import rerun as rr
rr.init("My Remote App", spawn=False)
rr.connect("10.147.17.44:9876")
```
