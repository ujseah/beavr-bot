from dataclasses import dataclass
from typing import List, Optional, Sequence, Tuple

# Interface telemetry/events flowing upstream to recorders/monitors.


@dataclass(frozen=True)
class RobotState:
    """Minimal robot state for feedback/recording.

    - joint_positions_rad: current joint angles.
    - ee_position_m, ee_orientation_xyzw: current end-effector pose.
    """

    timestamp_s: float
    joint_positions_rad: List[float]
    ee_position_m: Tuple[float, float, float]
    ee_orientation_xyzw: Tuple[float, float, float, float]


@dataclass(frozen=True)
class ErrorEvent:
    """Error surface communicated by the interface.

    - code: short code; message: human-readable detail.
    """

    timestamp_s: float
    code: str
    message: Optional[str] = None


@dataclass(frozen=True)
class CartesianState:
    """Typed cartesian state wrapper used by interfaces before publishing.

    This is an internal boundary type. When publishing, use ``to_dict()`` to
    preserve the existing wire format expected by downstream consumers
    (e.g. MultiRobotAdapter/BeavrBot), namely::

        {"cartesian_position": [x, y, z], "timestamp": <float>}
    """

    position_m: Tuple[float, float, float]
    timestamp_s: float

    def to_dict(self) -> dict:
        return {
            "cartesian_position": list(self.position_m),
            "timestamp": self.timestamp_s,
        }


@dataclass(frozen=True)
class CommandedCartesianState:
    """Typed commanded cartesian state wrapper.

    The commanded end-effector target is represented as 7 values in the order
    [x, y, z, qx, qy, qz, qw]. The published wire format is::

        {"commanded_cartesian_position": [..7..], "timestamp": <float>}
    """

    commanded_cartesian_position: Sequence[float]  # length 7
    timestamp_s: float

    def to_dict(self) -> dict:
        return {
            "commanded_cartesian_position": list(self.commanded_cartesian_position),
            "timestamp": self.timestamp_s,
        }


__all_= [
    "RobotState",
    "ErrorEvent",
    "CartesianState",
    "CommandedCartesianState",
]
