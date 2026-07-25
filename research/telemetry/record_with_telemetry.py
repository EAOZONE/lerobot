#!/usr/bin/env python
"""`lerobot-record`, with the telemetry robot types registered. Zero library edits.

Importing so_follower_telemetry runs its `@RobotConfig.register_subclass` decorators,
which is all draccus needs to resolve `--robot.type=so101_follower_telemetry`. The
wrapped `record()` then parses `sys.argv` exactly as the normal entrypoint does
(src/lerobot/configs/parser.py:286-320), so every flag `lerobot-record` accepts works
here unchanged.

Why a wrapper instead of an import line inside lerobot_record.py: `lerobot-record` is
an installed console script, so `sys.path[0]` is the bin directory and a bare
`import research.telemetry...` there would not resolve. Running this file puts
research/telemetry on sys.path[0] instead, and leaves src/lerobot untouched.

Usage (identical to lerobot-record apart from the robot type):
    python research/telemetry/record_with_telemetry.py \
        --robot.type=so101_follower_telemetry \
        --robot.port=/dev/ttyACM0 \
        --robot.id=my_follower \
        --robot.cameras='{ wrist: {type: opencv, index_or_path: 0, width: 640, height: 480, fps: 30}}' \
        --teleop.type=so101_leader \
        --teleop.port=/dev/ttyACM1 \
        --teleop.id=my_leader \
        --dataset.repo_id=${HF_USER}/slip-corpus \
        --dataset.single_task="Pick the marker and place it in the cup" \
        --dataset.num_episodes=5
"""

import so_follower_telemetry  # noqa: F401  -- import registers the robot config subclasses

from lerobot.scripts.lerobot_record import record
from lerobot.utils.import_utils import register_third_party_plugins


def main() -> None:
    register_third_party_plugins()
    record()


if __name__ == "__main__":
    main()
