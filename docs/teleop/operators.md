# Operators

Operators transform VR input into robot commands. They are defined in `beavr.teleop.components.operators` and inherit from the base `Operator` class.

An operator subscribes to hand keypoints, applies retargeting (see `xarm_base.py` for an example), and sends actions to a robot interface. Operators also publish state information which can be recorded.

Clean shutdown is handled via the `cleanup()` method which stops network subscribers and closes sockets.
