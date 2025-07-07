import draccus
import logging

from dataclasses import dataclass, field
from typing import Any
from beavr.configs_teleop import constants as CONST
from beavr.configs_teleop import TeleopRobotConfig
_CONFIGS_PKG = "beavr.configs_teleop"


# -----------------------------------------------------------------------------
# Dataclass-based configuration (flattened network + teleop + robot)
# -----------------------------------------------------------------------------
@dataclass
class NetworkConfig:
    host_address: str = CONST.HOST_ADDRESS

    left_xarm_ip: str = CONST.LEFT_XARM_IP
    right_xarm_ip: str = CONST.RIGHT_XARM_IP

    # Consolidated publisher ports
    keypoint_stream_port: int = CONST.KEYPOINT_STREAM_PORT
    control_stream_port: int = CONST.CONTROL_STREAM_PORT
    robot_state_port: int = CONST.ROBOT_STATE_PORT
    robot_command_port: int = CONST.ROBOT_COMMAND_PORT

    # Input ports
    right_hand_oculus_receiver_port: int = CONST.RIGHT_HAND_OCULUS_RECEIVER_PORT
    resolution_button_port: int = CONST.RESOLUTION_BUTTON_PORT
    teleop_reset_port: int = CONST.TELEOP_RESET_PORT

    # Left-hand specific
    left_hand_oculus_receiver_port: int = CONST.LEFT_HAND_OCULUS_RECEIVER_PORT

    # Misc
    cam_port_offset: int = CONST.CAM_PORT_OFFSET
    oculus_graph_port: int = CONST.OCULUS_GRAPH_PORT
    deployment_port: int = CONST.DEPLOYMENT_PORT
    sim_image_port: int = CONST.SIM_IMAGE_PORT
    fish_eye_cam_port_offset: int = CONST.FISH_EYE_CAM_PORT_OFFSET

    pre_action_thumb_ee_position_port: int = CONST.PRE_ACTION_THUMB_EE_POSITION_PORT
    post_action_thumb_ee_position_port: int = CONST.POST_ACTION_THUMB_EE_POSITION_PORT

    gripper_publish_port_right: int = CONST.GRIPPER_PUBLISH_PORT_RIGHT
    gripper_publish_port_left: int = CONST.GRIPPER_PUBLISH_PORT_LEFT

    cartesian_publisher_port: int = CONST.CARTESIAN_PUBLISHER_PORT
    joint_publisher_port: int = CONST.JOINT_PUBLISHER_PORT
    cartesian_command_publisher_port: int = CONST.CARTESIAN_COMMAND_PUBLISHER_PORT

    cartesian_publisher_port_left: int = CONST.CARTESIAN_PUBLISHER_PORT_LEFT
    joint_publisher_port_left: int = CONST.JOINT_PUBLISHER_PORT_LEFT
    cartesian_command_publisher_port_left: int = CONST.CARTESIAN_COMMAND_PUBLISHER_PORT_LEFT

    resolution_button_publish_port: int = CONST.RESOLUTION_BUTTON_PUBLISH_PORT
    teleop_reset_publish_port: int = CONST.TELEOP_RESET_PUBLISH_PORT

    unified_data_port: int = CONST.UNIFIED_DATA_PORT
    keypoint_transform_port: int = CONST.KEYPOINT_TRANSFORM_PORT
    left_keypoint_transform_port: int = CONST.LEFT_KEYPOINT_TRANSFORM_PORT


@dataclass
class TeleopConfig:
    # Teleop flags
    operate: bool = True
    robot_interface: bool = True
    sim_env: bool = False
    run_xela: bool = False
    visualize_xela: bool = False
    visualize_right_2d: bool = False
    visualize_right_3d: bool = False
    visualize_right_dir: bool = False

    # Nested network config
    network: NetworkConfig = field(default_factory=NetworkConfig)

    # Import-path identifying which generated robot config dataclass to use.
    robot_name: str | None = None  # no default → must be passed via CLI

    # Built robot structure (DotDict of component instances) – populated at
    # runtime in ``__post_init__``.
    robot: Any = field(init=False)

    def __post_init__(self):
        # Ensure robot_name is provided
        if not getattr(self, 'robot_name', None):
            raise ValueError("robot_name must be provided in TeleopConfig. Example: --robot_name=leap_xarm_right")
        """Dynamically import and instantiate the selected robot configuration.

        The generated robot config dataclasses live in ``configs/<robot_name>_config.py`` and
        register themselves with ``TeleopRobotConfig`` when imported.  We therefore import the
        corresponding module first to ensure the registration side-effect has taken place before
        asking `TeleopRobotConfig` for the class.
        """

        import importlib

        # Import the auto-generated config module (e.g. ``configs/leap_xarm_right_config.py``).
        try:
            importlib.import_module(f"{_CONFIGS_PKG}.{self.robot_name}_config")
        except ModuleNotFoundError as exc:
            raise ValueError(
                f"Could not find auto-generated config module for robot '{self.robot_name}'."
            ) from exc

        # Retrieve the registered dataclass for the requested robot and instantiate it.
        cfg_cls = TeleopRobotConfig.get_choice_class(self.robot_name)
        self.robot = cfg_cls()

    # ------------------------------------------------------------------
    # Convenience attribute delegation
    # ------------------------------------------------------------------
    def __getattr__(self, item):  # noqa: D401
        """Delegate unknown attribute lookups to the nested `network` config.

        This allows the remainder of the codebase (e.g., `TeleOperator`) to
        keep using the flattened attribute style (``configs.host_address``)
        while we preserve a clean separation between *teleop* and *network*
        configuration in this dataclass.
        """
        try:
            return getattr(self.network, item)
        except AttributeError as exc:
            raise AttributeError(item) from exc

# -----------------------------------------------------------------------------
# Teleop execution helpers
# -----------------------------------------------------------------------------

def _setup_root_logger(level: int = logging.DEBUG):
    """Configure the *root* logger only once (no-op if already configured)."""
    root = logging.getLogger()
    if root.handlers:
        # A configuration already exists – just raise the level if needed.
        if root.level > level:
            root.setLevel(level)
        return

    logging.basicConfig(
        level=level,
        format="[%(levelname)s] %(asctime)s %(processName)s %(name)s: %(message)s",
        datefmt="%H:%M:%S",
    )


def run_teleop(configs):
    """Run the teleoperation system with given configs."""

    # Make sure we see *all* debug output from the refactored networking stack
    _setup_root_logger(logging.DEBUG)

    # Lazy-import to avoid incurring heavy dependencies during mere CLI parsing
    from beavr.components import TeleOperator  # pylint: disable=import-error,cyclic-import

    teleop = TeleOperator(configs)
    processes = teleop.get_processes()

    try:
        # Start all processes
        for process in processes:
            process.start()

        # Wait for all processes to complete
        while any(p.is_alive() for p in processes):
            for p in processes:
                p.join(timeout=0.1)  # Short timeout to allow checking KeyboardInterrupt

    except KeyboardInterrupt:
        print("\nShutdown requested...")
        
        # First send stop signal to all processes
        for p in processes:
            if p.is_alive():
                p.terminate()
        
        # Then wait for them to finish with timeout
        for p in processes:
            p.join(timeout=2.0)
            
        # Force kill any remaining processes
        for p in processes:
            if p.is_alive():
                print(f"Process {p.name} did not terminate gracefully - force killing")
                p.kill()
                p.join(timeout=1.0)
    
    finally:
        # Final cleanup
        for p in processes:
            if p.is_alive():
                try:
                    p.terminate()
                    p.join(timeout=0.5)
                except Exception as e:
                    print(f"Error cleaning up process {p.name}: {e}")
        
        print("Teleop shutdown complete")

# -----------------------------------------------------------------------------
# CLI entry-point using Draccus
# -----------------------------------------------------------------------------
@draccus.wrap()
def main(cli_cfg: TeleopConfig):
    run_teleop(cli_cfg)


__all__ = [
    "TeleopConfig",
    "NetworkConfig",
    "run_teleop",
    "main",
] 