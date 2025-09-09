from dataclasses import dataclass
from typing import List, Optional, Tuple

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


__all__ = [
    "RobotState",
    "ErrorEvent",
]


