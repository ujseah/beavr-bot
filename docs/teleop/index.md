# Teleoperation Stack

The teleoperation stack is responsible for streaming VR signals, controlling the robot interfaces and collecting data. It is centered around the modules in `beavr.teleop`.

The stack is launched from `beavr/teleop/main.py` and can be configured entirely through the command line or YAML files. The configuration combines:

- **Networking** – IP addresses and ZMQ ports for communication
- **Control** – frequency settings and teleoperation flags
- **Interfaces** – robot specific adapters that translate commands
- **Operators** – processes that retarget VR input to robot actions

Use `python -m beavr.teleop.main --robot_name=leap,xarm7 --laterality=bimanual` to start the default bimanual setup.
