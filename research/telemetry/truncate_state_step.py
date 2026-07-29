#!/usr/bin/env python
"""Hide the telemetry tail of `observation.state` from the policy.

`so_follower_telemetry` records a 30-dim `observation.state` (pos, load, current, vel, volt).
The detectors want all of it. SmolVLA must not have it, for two reasons:

  - a 30-dim state no longer matches `lerobot/smolvla_base`'s pretrained state
    projection;
  - more importantly, a policy that consumes load and current is a different policy.
    RQ2 asks how telemetry-based detectors compare against model-internal ones *for a
    given policy*. If the policy itself sees telemetry, its internal representations
    already encode the signal under test and the comparison is confounded.

This step slices `observation.state` back to its first `keep` dims. Because
`so_follower_telemetry` orders the vector positions-first, `state[:6]` is bit-identical
to what a plain `so101_follower` recording would have produced -- so a policy trained
through this step is trained on exactly the baseline observation.

Drop the step to run the telemetry-conditioned ablation.

Usage in a training/inference pipeline:

    from truncate_state_step import TruncateStateStep
    pipeline = PolicyProcessorPipeline(steps=[TruncateStateStep(keep=6), ...])
"""

from dataclasses import dataclass

import numpy as np
import torch

from lerobot.configs.types import PipelineFeatureType, PolicyFeature
from lerobot.processor import ObservationProcessorStep, ProcessorStepRegistry
from lerobot.utils.constants import OBS_STATE


@dataclass
@ProcessorStepRegistry.register("truncate_state")
class TruncateStateStep(ObservationProcessorStep):
    """Keep only the first `keep` dimensions of `observation.state`.

    Attributes:
        keep: number of leading dimensions to retain. 6 = the SO-101's joint positions.
    """

    keep: int = 6

    def get_config(self) -> dict[str, int]:
        """Persist ``keep`` explicitly; it is a research-arm identity, not a disposable default."""
        return {"keep": self.keep}

    def observation(self, observation: dict) -> dict:
        state = observation.get(OBS_STATE)
        if state is None:
            return observation

        # Already narrow enough: a plain so101_follower recording passes through
        # untouched, so the same pipeline works against both dataset generations.
        if state.shape[-1] <= self.keep:
            return observation

        new_observation = dict(observation)
        if isinstance(state, (torch.Tensor, np.ndarray)):
            new_observation[OBS_STATE] = state[..., : self.keep]
        else:
            raise TypeError(f"{OBS_STATE} must be a tensor or ndarray, got {type(state)}")
        return new_observation

    def transform_features(
        self, features: dict[PipelineFeatureType, dict[str, PolicyFeature]]
    ) -> dict[PipelineFeatureType, dict[str, PolicyFeature]]:
        """Shrink the declared `observation.state` shape to match what `observation` emits."""
        obs_features = features.get(PipelineFeatureType.OBSERVATION, {})
        if OBS_STATE not in obs_features:
            return features

        original = obs_features[OBS_STATE]
        if original.shape[0] <= self.keep:
            return features

        obs_features[OBS_STATE] = PolicyFeature(
            type=original.type, shape=(self.keep,) + tuple(original.shape[1:])
        )
        return features
