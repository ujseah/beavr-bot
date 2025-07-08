#!/usr/bin/env python3
"""
Main entry point for Beavr Teleop system.

This module provides the CLI interface and main execution logic for the teleoperation system.
Uses the structured configuration system with automatic CLI flag generation via Draccus.
"""

import draccus
import logging
import importlib
import os
import yaml
from typing import Any
from dataclasses import dataclass, field

from beavr.teleop.configs.constants.models import TeleopConfig
from beavr.teleop.configs.robots import TeleopRobotConfig

logger = logging.getLogger(__name__)
_CONFIGS_PKG = "beavr.teleop.configs.robots"


@dataclass
class MainConfig:
    """Main configuration combining structured teleop config with robot selection."""
    
    # Use our new structured config as the base
    teleop: TeleopConfig = field(default_factory=TeleopConfig)
    
    # Robot selection - must be provided via CLI
    robot_name: str = ""

    # Optional config file override
    config_file: str = "config/dev.yaml"
    
    # Data storage configuration
    storage_path: str = "data/recordings"
    
    # Built robot structure (populated at runtime)
    robot: Any = field(init=False)
    
    def __post_init__(self):
        """Initialize robot configuration based on robot_name."""
        if not self.robot_name or self.robot_name == "":
            raise ValueError(
                "robot_name must be provided. Example: --robot_name=leap_xarm_right\n"
                "Available robots: leap_pybullet, leap_xarm_right, etc."
            )
        
        # Import the auto-generated config module
        try:
            importlib.import_module(f"{_CONFIGS_PKG}.{self.robot_name}_config")
        except ModuleNotFoundError as exc:
            raise ValueError(
                f"Could not find auto-generated config module for robot '{self.robot_name}'. "
                f"Available configs in {_CONFIGS_PKG}/"
            ) from exc
        
        # Retrieve and instantiate the registered robot config
        cfg_cls = TeleopRobotConfig.get_choice_class(self.robot_name)
        self.robot = cfg_cls()
        
        logger.info(f"✅ Loaded robot configuration: {self.robot_name}")
    
    # Convenience attribute delegation for backward compatibility
    def __getattr__(self, item):
        """Delegate unknown attributes to teleop config for backward compatibility."""
        try:
            return getattr(self.teleop, item)
        except AttributeError:
            # Try teleop.network for flattened network access
            try:
                return getattr(self.teleop.network, item)
            except AttributeError as exc:
                raise AttributeError(f"'{self.__class__.__name__}' has no attribute '{item}'") from exc



def _load_yaml_config(config_file: str) -> dict:
    """
    Load YAML configuration file with error handling.
    
    Args:
        config_file: Path to YAML configuration file
        
    Returns:
        Dictionary of configuration overrides
    """
    if not os.path.exists(config_file):
        logger.warning(f"⚠️  Config file not found: {config_file} - using defaults")
        return {}
    
    try:
        with open(config_file, 'r') as f:
            config_data = yaml.safe_load(f)
            if config_data is None:
                return {}
            logger.info(f"📄 Loaded config overrides from: {config_file}")
            return config_data
    except yaml.YAMLError as e:
        logger.error(f"❌ Failed to parse YAML config {config_file}: {e}")
        return {}
    except Exception as e:
        logger.error(f"❌ Failed to load config {config_file}: {e}")
        return {}


def _setup_root_logger(level: int = logging.DEBUG):
    """Configure the root logger only once (no-op if already configured)."""
    root = logging.getLogger()
    if root.handlers:
        # Configuration already exists – just raise the level if needed
        if root.level > level:
            root.setLevel(level)
        return

    logging.basicConfig(
        level=level,
        format="[%(levelname)s] %(asctime)s %(processName)s %(name)s: %(message)s",
        datefmt="%H:%M:%S",
    )


def run_teleop(config: MainConfig):
    """Run the teleoperation system with given configuration."""
    
    # Setup logging
    _setup_root_logger(logging.DEBUG)
    
    logger.info("🚀 Starting Beavr Teleop System")
    logger.info(f"📡 Network host: {config.teleop.network.host_address}")
    logger.info(f"🎮 Operation mode: {'ENABLED' if config.teleop.flags.operate else 'DISABLED'}")
    logger.info(f"🎯 Simulation mode: {'ENABLED' if config.teleop.flags.sim_env else 'DISABLED'}")
    logger.info(f"🤖 Robot: {config.robot_name}")
    
    from beavr.teleop.components import TeleOperator  # pylint: disable=import-error,cyclic-import

    # Initialize the teleoperator with structured config
    teleop = TeleOperator(config)
    processes = teleop.get_processes()

    try:
        # Start all processes
        logger.info(f"🔄 Starting {len(processes)} teleop processes...")
        for process in processes:
            process.start()
            logger.debug(f"  ✅ Started process: {process.name}")

        # Wait for all processes to complete
        logger.info("✨ All processes started. Press Ctrl+C to stop.")
        while any(p.is_alive() for p in processes):
            for p in processes:
                p.join(timeout=0.1)  # Short timeout to allow checking KeyboardInterrupt

    except KeyboardInterrupt:
        logger.info("\n🛑 Shutdown requested...")
        
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
                logger.warning(f"Process {p.name} did not terminate gracefully - force killing")
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
                    logger.error(f"Error cleaning up process {p.name}: {e}")
        
        logger.info("🏁 Teleop shutdown complete")


@draccus.wrap()
def main(cfg: MainConfig):
    """
    Main entry point for Beavr Teleop system.
    
    This function is wrapped with Draccus to automatically generate CLI flags for all
    configuration parameters. Configuration precedence (highest to lowest):
    1. CLI flags (via Draccus)
    2. YAML config file overrides  
    3. Default values
    
    Examples:
        # Basic usage with dev environment
        python -m beavr.teleop.main --robot_name=leap_xarm_right
        
        # Use production config
        python -m beavr.teleop.main --robot_name=leap_xarm_right --config_file=config/prod.yaml
        
        # Override network settings via CLI (highest priority)
        python -m beavr.teleop.main --robot_name=leap_pybullet --teleop.network.host_address=192.168.1.100
        
        # Enable simulation mode
        python -m beavr.teleop.main --robot_name=leap_pybullet --teleop.flags.sim_env=True
        
        # Override control parameters
        python -m beavr.teleop.main --robot_name=leap_xarm_right --teleop.control.vr_freq=60
        
        # Override multiple port settings
        python -m beavr.teleop.main --robot_name=leap_xarm_right \
            --teleop.ports.keypoint_stream_port=9000 \
            --teleop.ports.control_stream_port=9001
    """
    
    # Apply YAML configuration overrides
    # Note: CLI flags (from Draccus) already applied, YAML merges underneath
    yaml_overrides = _load_yaml_config(cfg.config_file)
    if yaml_overrides:
        logger.info(f"🔧 Applying YAML overrides from {cfg.config_file}")
        
        # Apply YAML overrides while preserving CLI flag precedence
        _apply_yaml_preserving_cli(cfg, yaml_overrides)
    
    run_teleop(cfg)


def _apply_section_override(target, yaml_obj, defaults, section_name):
    """
    Apply YAML overrides to a config section while preserving CLI flag precedence.
    
    Args:
        target: The target config object to modify
        yaml_obj: The YAML overrides dictionary for this section
        defaults: The default config object for comparison
        section_name: Section name for logging purposes
    """
    overrides = yaml_obj or {}
    
    for key, yaml_value in overrides.items():
        try:
            current = getattr(target, key)
            default = getattr(defaults, key)
            
            # If current value equals default, it wasn't overridden by CLI
            if current == default:
                setattr(target, key, yaml_value)
                logger.debug(f"📝 Applied YAML override: {section_name}.{key} = {yaml_value}")
            else:
                logger.debug(f"🚫 Skipped YAML override (CLI precedence): {section_name}.{key}")
                
        except AttributeError as e:
            logger.warning(f"⚠️  Unknown config key in YAML: {section_name}.{key} - {e}")


def _apply_yaml_preserving_cli(target_cfg: MainConfig, yaml_overrides: dict):
    """
    Apply YAML overrides while preserving CLI flag precedence.
    
    Uses _apply_section_override for each config section to maintain clean separation.
    """
    
    # Safety check - ensure teleop section exists
    teleop_overrides = yaml_overrides.get('teleop', {})
    if not teleop_overrides:
        logger.debug("No 'teleop' section found in YAML config")
        return
    
    # Create defaults for comparison
    defaults = TeleopConfig()
    
    # Apply each config section systematically
    sections = [
        (target_cfg.teleop.network, teleop_overrides.get('network'), defaults.network, 'teleop.network'),
        (target_cfg.teleop.ports, teleop_overrides.get('ports'), defaults.ports, 'teleop.ports'),
        (target_cfg.teleop.flags, teleop_overrides.get('flags'), defaults.flags, 'teleop.flags'),
        (target_cfg.teleop.control, teleop_overrides.get('control'), defaults.control, 'teleop.control'),
        (target_cfg.teleop.camera, teleop_overrides.get('camera'), defaults.camera, 'teleop.camera'),
    ]
    
    for target_section, yaml_section, default_section, section_name in sections:
        if yaml_section:  # Only process if section exists in YAML
            _apply_section_override(target_section, yaml_section, default_section, section_name)
    
    # Handle robot-level overrides if any (rare, but possible)
    robot_overrides = yaml_overrides.get('robot', {})
    if robot_overrides and hasattr(target_cfg, 'robot'):
        logger.debug("Found robot-level YAML overrides - applying directly")
        for key, value in robot_overrides.items():
            if hasattr(target_cfg.robot, key):
                setattr(target_cfg.robot, key, value)
                logger.debug(f"📝 Applied robot override: robot.{key} = {value}")
            else:
                logger.warning(f"⚠️  Unknown robot config key: robot.{key}")


if __name__ == "__main__":
    main()