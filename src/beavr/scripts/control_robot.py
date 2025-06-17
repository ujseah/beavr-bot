# Copyright 2024 The HuggingFace Inc. team. All rights reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
"""
Utilities to control a robot.

Useful to record a dataset, replay a recorded episode, run the policy on your robot
and record an evaluation dataset, and to recalibrate your robot if needed.

Examples of usage:

- Recalibrate your robot:
```bash
python lerobot/scripts/control_robot.py \
    --robot.type=so100 \
    --control.type=calibrate
```

- Unlimited teleoperation at highest frequency (~200 Hz is expected), to exit with CTRL+C:
```bash
python lerobot/scripts/control_robot.py \
    --robot.type=so100 \
    --robot.cameras='{}' \
    --control.type=teleoperate

# Add the cameras from the robot definition to visualize them:
python lerobot/scripts/control_robot.py \
    --robot.type=so100 \
    --control.type=teleoperate
```

- Unlimited teleoperation at a limited frequency of 30 Hz, to simulate data recording frequency:
```bash
python lerobot/scripts/control_robot.py \
    --robot.type=so100 \
    --control.type=teleoperate \
    --control.fps=30
```

- Record one episode in order to test replay:
```bash
python lerobot/scripts/control_robot.py \
    --robot.type=so100 \
    --control.type=record \
    --control.fps=30 \
    --control.single_task="Grasp a lego block and put it in the bin." \
    --control.repo_id=$USER/koch_test \
    --control.num_episodes=1 \
    --control.push_to_hub=True
```

- Visualize dataset:
```bash
python lerobot/scripts/visualize_dataset.py \
    --repo-id $USER/koch_test \
    --episode-index 0
```

- Replay this test episode:
```bash
python lerobot/scripts/control_robot.py replay \
    --robot.type=so100 \
    --control.type=replay \
    --control.fps=30 \
    --control.repo_id=$USER/koch_test \
    --control.episode=0
```

- Record a full dataset in order to train a policy, with 2 seconds of warmup,
30 seconds of recording for each episode, and 10 seconds to reset the environment in between episodes:
```bash
python lerobot/scripts/control_robot.py record \
    --robot.type=so100 \
    --control.type=record \
    --control.fps 30 \
    --control.repo_id=$USER/koch_pick_place_lego \
    --control.num_episodes=50 \
    --control.warmup_time_s=2 \
    --control.episode_time_s=30 \
    --control.reset_time_s=10
```

- For remote controlled robots like LeKiwi, run this script on the robot edge device (e.g. RaspBerryPi):
```bash
python lerobot/scripts/control_robot.py \
  --robot.type=lekiwi \
  --control.type=remote_robot
```

**NOTE**: You can use your keyboard to control data recording flow.
- Tap right arrow key '->' to early exit while recording an episode and go to resseting the environment.
- Tap right arrow key '->' to early exit while resetting the environment and got to recording the next episode.
- Tap left arrow key '<-' to early exit and re-record the current episode.
- Tap escape key 'esc' to stop the data recording.
This might require a sudo permission to allow your terminal to monitor keyboard events.

**NOTE**: You can resume/continue data recording by running the same data recording command and adding `--control.resume=true`.

- Train on this dataset with the ACT policy:
```bash
python lerobot/scripts/train.py \
  --dataset.repo_id=${HF_USER}/koch_pick_place_lego \
  --policy.type=act \
  --output_dir=outputs/train/act_koch_pick_place_lego \
  --job_name=act_koch_pick_place_lego \
  --device=cuda \
  --wandb.enable=true
```

- Run the pretrained policy on the robot:
```bash
python lerobot/scripts/control_robot.py \
    --robot.type=so100 \
    --control.type=record \
    --control.fps=30 \
    --control.single_task="Grasp a lego block and put it in the bin." \
    --control.repo_id=$USER/eval_act_koch_pick_place_lego \
    --control.num_episodes=10 \
    --control.warmup_time_s=2 \
    --control.episode_time_s=30 \
    --control.reset_time_s=10 \
    --control.push_to_hub=true \
    --control.policy.path=outputs/train/act_koch_pick_place_lego/checkpoints/080000/pretrained_model
```
"""

import logging
import os
import time
import sys
import signal
import atexit
from dataclasses import asdict
from pprint import pformat
import subprocess

# Add project root to Python path
project_root = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..", ".."))
sys.path.insert(0, project_root)

# from safetensors.torch import load_file, save_file
from beavr.common.datasets.lerobot_dataset import LeRobotDataset
from beavr.common.policies.factory import make_policy
from beavr.common.robot_devices.control_configs import (
    ControlPipelineConfig,
    RecordControlConfig,
    ReplayControlConfig,
    TeleoperateControlConfig,
)
from beavr.common.robot_devices.control_utils import (
    init_keyboard_listener,
    log_control_info,
    record_episode,
    reset_environment,
    sanity_check_dataset_name,
    sanity_check_dataset_robot_compatibility,
    stop_recording,
    warmup_record,
)
from beavr.common.robot_devices.robots.utils import Robot, make_robot_from_config
from beavr.common.robot_devices.utils import busy_wait, safe_disconnect
from beavr.common.utils.utils import init_logging, log_say
from beavr.configs import parser


########################################################################################
# Control modes
########################################################################################

_teleop_process = None

def cleanup_processes():
    """Clean up any running processes when the script exits."""
    global _teleop_process
    if _teleop_process:
        logging.info("Cleaning up teleop process...")
        try:
            # Try graceful termination first
            _teleop_process.terminate()
            # Wait up to 3 seconds for process to terminate
            for _ in range(30):
                if _teleop_process.poll() is not None:
                    break
                time.sleep(0.1)
            # If process hasn't terminated, force kill it
            if _teleop_process.poll() is None:
                logging.warning("Teleop process didn't terminate gracefully, force killing...")
                _teleop_process.kill()
                _teleop_process.wait()
        except Exception as e:
            logging.error(f"Error cleaning up teleop process: {e}")
        _teleop_process = None

def signal_handler(signum, frame):
    """Handle termination signals by cleaning up processes."""
    logging.info(f"Received signal {signum}, cleaning up...")
    cleanup_processes()
    sys.exit(0)

# Register cleanup functions
atexit.register(cleanup_processes)
signal.signal(signal.SIGINT, signal_handler)
signal.signal(signal.SIGTERM, signal_handler)

def start_teleop_process(wait_for_exit=False):
    """Start teleop.py as a subprocess."""
    global _teleop_process
    if _teleop_process and _teleop_process.poll() is None:
        logging.info("Teleop process is already running.")
        return

    teleop_script_path = os.path.join(project_root, "teleop.py")
    if not os.path.exists(teleop_script_path):
        logging.error(f"Teleoperation script not found at '{teleop_script_path}'. Cannot start teleop.")
        return

    logging.info(f"Starting teleoperation process from: {teleop_script_path}")
    try:
        _teleop_process = subprocess.Popen([
            sys.executable,
            teleop_script_path,
            "robot=leap_xarm_right"
        ])
        
        # Give the teleop system a moment to initialize all ZMQ publishers
        logging.info("Waiting 1s for teleoperation system to initialize...")
        time.sleep(1)

        if wait_for_exit:
            try:
                _teleop_process.wait()
            except KeyboardInterrupt:
                logging.info("Teleoperation interrupted by user.")
            finally:
                stop_teleop_process()
            
    except Exception as e:
        logging.error(f"Failed to start teleoperation process: {e}")
        _teleop_process = None

def stop_teleop_process():
    """Stop the teleoperation subprocess if it's running."""
    global _teleop_process
    if _teleop_process and _teleop_process.poll() is None:
        logging.info("Stopping teleoperation process...")
        _teleop_process.terminate()
        try:
            _teleop_process.wait(timeout=2)
        except subprocess.TimeoutExpired:
            logging.warning("Teleop process did not terminate gracefully, killing it.")
            _teleop_process.kill()
            _teleop_process.wait()
    _teleop_process = None

@safe_disconnect
def teleoperate():
    """Use the BeaVR teleoperation system by running teleop.py and waiting for it to complete."""
    start_teleop_process(wait_for_exit=True)


@safe_disconnect
def record(
    robot: Robot,
    cfg: RecordControlConfig,
) -> LeRobotDataset:
    # When recording, the teleoperation system is assumed to be already running.
    # This function is now only responsible for the recording logic itself.
    try:
        # TODO: Add option to record logs
        if cfg.resume:
            dataset = LeRobotDataset(
                cfg.repo_id,
                root=cfg.root,
            )
            if len(robot.cameras) > 0:
                dataset.start_image_writer(
                    num_processes=cfg.num_image_writer_processes,
                    num_threads=cfg.num_image_writer_threads_per_camera * len(robot.cameras),
                )
            sanity_check_dataset_robot_compatibility(dataset, robot, cfg.fps, cfg.video)
        else:
            # Create empty dataset or load existing saved episodes
            sanity_check_dataset_name(cfg.repo_id, cfg.policy)
            dataset = LeRobotDataset.create(
                cfg.repo_id,
                cfg.fps,
                root=cfg.root,
                robot=robot,
                use_videos=cfg.video,
                image_writer_processes=cfg.num_image_writer_processes,
                image_writer_threads=cfg.num_image_writer_threads_per_camera * len(robot.cameras),
            )

        # Load pretrained policy
        policy = None if cfg.policy is None else make_policy(cfg.policy, ds_meta=dataset.meta)

        listener, events = init_keyboard_listener()

        # Execute a few seconds without recording to:
        # 1. teleoperate the robot to move it in starting position if no policy provided,
        # 2. give times to the robot devices to connect and start synchronizing,
        # 3. place the cameras windows on screen
        enable_teleoperation = policy is None
        log_say("Warmup record", cfg.play_sounds)
        warmup_record(robot, events, enable_teleoperation, cfg.warmup_time_s, cfg.display_data, cfg.fps)

        recorded_episodes = 0
        while True:
            if recorded_episodes >= cfg.num_episodes:
                break

            log_say(f"Recording episode {dataset.num_episodes}", cfg.play_sounds)
            record_episode(
                robot=robot,
                dataset=dataset,
                events=events,
                episode_time_s=cfg.episode_time_s,
                display_data=cfg.display_data,
                policy=policy,
                fps=cfg.fps,
                single_task=cfg.single_task,
            )

            # Execute a few seconds without recording to give time to manually reset the environment
            # TODO: add an option to enable teleoperation during reset
            # Skip reset for the last episode to be recorded
            if not events["stop_recording"] and (
                (recorded_episodes < cfg.num_episodes - 1) or events["rerecord_episode"]
            ):
                log_say("Reset the environment", cfg.play_sounds)
                reset_environment(robot, events, cfg.reset_time_s, cfg.fps)

            if events["rerecord_episode"]:
                log_say("Re-record episode", cfg.play_sounds)
                events["rerecord_episode"] = False
                events["exit_early"] = False
                dataset.clear_episode_buffer()
                continue

            dataset.save_episode()
            recorded_episodes += 1

            if events["stop_recording"]:
                break

        log_say("Stop recording", cfg.play_sounds, blocking=True)
        stop_recording(robot, listener, cfg.display_data)

        if cfg.push_to_hub:
            dataset.push_to_hub(tags=cfg.tags, private=cfg.private)

        log_say("Exiting", cfg.play_sounds)
        return dataset
    finally:
        # Cleanup is now handled by the main control_robot function
        pass


@safe_disconnect
def replay(
    robot: Robot,
    cfg: ReplayControlConfig,
):
    # TODO: refactor with control_loop, once `dataset` is an instance of LeRobotDataset
    # TODO: Add option to record logs

    dataset = LeRobotDataset(cfg.repo_id, root=cfg.root, episodes=[cfg.episode])
    actions = dataset.hf_dataset.select_columns("action")

    if not robot.is_connected:
        robot.connect()

    log_say("Replaying episode", cfg.play_sounds, blocking=True)
    for idx in range(dataset.num_frames):
        start_episode_t = time.perf_counter()

        action = actions[idx]["action"]
        robot.send_action(action)

        dt_s = time.perf_counter() - start_episode_t
        busy_wait(1 / cfg.fps - dt_s)

        dt_s = time.perf_counter() - start_episode_t
        log_control_info(robot, dt_s, fps=cfg.fps)


@parser.wrap()
def control_robot(cfg: ControlPipelineConfig):
    init_logging()
    logging.info(pformat(asdict(cfg)))

    # Start the teleoperation process first if needed.
    # This is crucial because it sets up ZMQ publishers that the robot adapter will need to connect to.
    if isinstance(cfg.control, (RecordControlConfig, TeleoperateControlConfig)):
        start_teleop_process()
        # Give the teleop system a moment to initialize all ZMQ publishers and for the robot to settle.
        logging.info("Waiting 5s for teleoperation system to initialize before robot connection...")
        time.sleep(5)

    # Pass the controller to the robot adapter
    robot = make_robot_from_config(cfg.robot)

    try:
        if not robot.is_connected:
            robot.connect()

        if isinstance(cfg.control, TeleoperateControlConfig):
            # The teleop process was already started, just wait for it to complete.
            global _teleop_process
            if _teleop_process:
                logging.info("Waiting for teleoperation process to complete...")
                _teleop_process.wait()

        elif isinstance(cfg.control, RecordControlConfig):
            record(robot, cfg.control)
        elif isinstance(cfg.control, ReplayControlConfig):
            replay(robot, cfg.control)
    except KeyboardInterrupt:
        logging.info("Main control loop interrupted by user.")
    finally:
        # This block ensures cleanup happens on normal exit or interrupt.
        logging.info("Starting cleanup...")
        if robot.is_connected:
            logging.info("Disconnecting robot...")
            robot.disconnect()
        # Always try to stop the teleop process on exit
        stop_teleop_process()
        logging.info("Cleanup complete. Exiting.")


if __name__ == "__main__":
    control_robot()