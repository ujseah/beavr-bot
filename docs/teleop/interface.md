# Interfaces

Robot specific adapters live under `beavr.teleop.interfaces`. Each adapter implements the `RobotWrapper` API providing methods to query state and send commands.

Available interfaces include:

- `LeapHandRobot` – communicates with a simulated or real Leap hand
- `XArm7Robot` – interface for xArm arms
- `RX1RightRobot` – interface for the RX-1 humanoid arm

Interfaces are selected through the `--robot_name` argument. Multiple robots can be combined by comma separating their names.
