python src/beavr/scripts/control_robot.py \
    --robot.type=multi_robot_adapter \
    --control.type=record \
    --control.video=true \
    --control.fps=30 \
    --control.num_episodes=4 \
    --control.warmup_time_s=5 \
    --control.episode_time_s=30 \
    --control.reset_time_s=30 \
    --control.repo_id=aposadasn/lx7r_pickup_test_dataset \
    --control.single_task="Move the right xarm7 to the target position" \
    # --control.resume=true