# Networking

All runtime communication uses ZeroMQ. IP addresses and ports are defined by `beavr.teleop.configs.constants` and exposed through the `NetworkConfig` and `PortsConfig` dataclasses.

Typical ports used in a default session include:

| Purpose | Config Attribute | Default |
|---------|-----------------|---------|
| Hand keypoint stream | `keypoint_stream_port` | 8000 |
| Robot commands | `control_stream_port` | 8001 |
| Robot state publish | `robot_state_port` | 8002 |

The `HandshakeCoordinator` class (see `beavr.teleop.utils.network`) provides reliable delivery for stop/resume events by requiring acknowledgements from all registered subscribers.

You can customize any address or port via command line:

```bash
python -m beavr.teleop.main --teleop.network.host_address=192.168.1.50 --teleop.ports.control_stream_port=9001
```
