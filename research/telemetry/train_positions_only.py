#!/usr/bin/env python
"""Run LeRobot training with a six-position policy input (Arms 1 and 2).

The dataset stays 30 dimensional. Use standard ``lerobot-train`` for the intentionally
telemetry-conditioned Arms 3 and 4.
"""

from telemetry_policy_bridge import install_positions_only_training

from lerobot.scripts import lerobot_train


def main() -> None:
    install_positions_only_training(lerobot_train, keep=6)
    lerobot_train.main()


if __name__ == "__main__":
    main()
