python src/beavr/scripts/control_robot.py \
    --robot.type=multi_robot_adapter \
    --control.type=record \
    --control.video=true \
    --control.fps=30 \
    --control.num_episodes=10 \
    --control.warmup_time_s=5 \
    --control.episode_time_s=30 \
    --control.reset_time_s=5 \
    --control.repo_id=arclabmit/lx7r_flip_cube_dataset \
    --control.single_task="Flip cube" \
    --control.resume=false
    