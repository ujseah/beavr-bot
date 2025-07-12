# Control Loop

Operators retrieve VR keypoints and compute target robot actions. The `TeleopControlConfig` dataclass exposes frequency parameters such as `vr_freq` for VR input rate and `recorder_freq` for dataset recording.

Each operator runs a loop similar to:

```python
while True:
    pose = hand_subscriber.get()
    command = solver.retarget(pose)
    robot.send_action(command)
```

Use `--teleop.control.vr_freq=60` to change the VR polling rate from the command line.
