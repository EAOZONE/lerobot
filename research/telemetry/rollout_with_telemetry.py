#!/usr/bin/env python
"""Run ``lerobot-rollout`` with telemetry robot and alignment-sidecar support.

Use this wrapper for autonomous policy evaluation. ``record_with_telemetry.py`` is only
for teleoperated demonstrations in the current LeRobot API and cannot accept a policy.
"""

import so_follower_telemetry  # noqa: F401 -- registers telemetry robot configs
from alignment_sidecar import record_alignment_sidecars
from truncate_state_step import TruncateStateStep as _TruncateStateStep  # noqa: F401

from lerobot.scripts.lerobot_rollout import rollout
from lerobot.utils.import_utils import register_third_party_plugins


def main() -> None:
    register_third_party_plugins()
    with record_alignment_sidecars():
        rollout()


if __name__ == "__main__":
    main()
