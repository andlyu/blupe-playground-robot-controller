---
pretty_name: Public YAM runs
language:
- en
tags:
- robotics
- bimanual
- yam
- lerobot
configs:
- config_name: default
  data_files:
  - split: train
    path: data/*.parquet
- config_name: lerobot
  data_files:
  - split: train
    path: data/chunk-*/*.parquet
---

# Public-YAM-runs

Physical bimanual YAM episodes recorded by the BluPe operator station.
Each run adds an episode to this repository. Failed, interrupted, stopped and
timed-out runs are retained and labeled; these are **not all successful demonstrations**.
A model saying done is not independently verified task success.

## Loading

```python
from datasets import load_dataset
runs = load_dataset("andlyu/Public-YAM-runs", split="train")
usable = runs.filter(lambda row: row["training_valid"])
```

The default configuration preserves the original Image/Parquet archive. The
`lerobot` configuration and `meta/` directory provide an additional LeRobot
v2.1 layout with separate camera videos. Each atomic commit adds an episode
and its metadata. Previously uploaded archive files and replay URLs are retained.

`videos/ep_*.mp4` contains a 10 FPS H.264 view arranged **left | top | right**.
Video playback follows the episode timeline, labels stale camera frames, and
shows missing-frame gaps rather than silently skipping them. It is a viewing
artifact; the original images and timestamps remain in the Parquet rows.
The local runner's **Save video** prefers its browser-recorded replay when available, otherwise this episode's MP4. **Save log**
downloads a separate local ZIP with exact model request/response bodies, model
input images, interaction events and errors. Model logs are not uploaded here.

## Samples

- `episode_id`, `frame_index`, `timestamp`: episode identity, sample index and
  seconds relative to the recording start on the Jetson monotonic clock.
- `left_image`, `right_image`, `top_image`: JPEGs of the corresponding camera.
  The station's capture-time overlay is retained in the pixels.
- `measured_joints`: 12 measured joint positions, **radians**, left joints 0–5
  followed by right joints 0–5. `measured_grippers`: left/right normalized
  positions, 0 closed and 1 open.
- `action_joints`, `action_grippers`: the most recent targets actually dispatched
  by the driver, in the same order/units. They are not substituted for feedback.
  `action_timestamp` identifies that dispatch relative to recording start.
  `action_valid=false` means no known command; its zero placeholders are not actions.
- Camera timestamps, sequence numbers and frame ages are retained per view;
  `camera_skew_s` records inter-camera timing difference.
- `training_valid`: known action and all three frames at most 150 ms old at the
  measured sample time. Retained invalid rows should be filtered for training.
- `task`: task instruction from the API handoff, or null if unavailable.
- `outcome`: station termination reason; `session_api_stop` does not distinguish
  success from a runner/operator stop. `session_timeout` is timeout, and
  `session_api_disconnect` is interrupted communication.

The nominal sampling rate is 10 Hz. USB cameras are **software time-aligned**,
not hardware-triggered; timestamps are host capture-return timestamps rather
than sensor exposure timestamps. Do not assume uniform intervals or perfect
synchronization. Missing samples and capture errors are counted in the episode
manifest. Do not silently interpolate them into measured demonstrations.

Recording ends when policy authority is revoked. Timeout parking and other
subsequent recovery motion are excluded from policy samples. Local recordings
are retained if publication fails, and the uploader retries independently of
arm control. Simulated episodes and model transcripts are not published here.

## Provenance and limitations

All recorded poses are robot joint-space state, not calibrated world poses.
Camera role mapping is left=10, right=4, top=16 on this station; camera calibration
and image-space task success are not supplied. Review outcome, timing validity
and dataset quality before training. No success-only filtering is implied by
the `train` split name.

## LeRobot v2.1 representation

Canonical files are `data/chunk-000/episode_000000.parquet` and
`videos/chunk-000/observation.images.{left,top,right}/episode_000000.mp4`,
with path templates, dimensions, counts, tasks and statistics in `meta/`.
`meta/blupe_source_episodes.jsonl` maps stable numeric indices to original
episode IDs, source hashes, outcomes and recorder quality counts.

State and action are float32 vectors with this exact order:
**left joints 0–5, left gripper, right joints 0–5, right gripper**.
Joint units are radians; grippers are normalized 0–1. State is measured feedback;
action is the most recently dispatched target. Unknown actions retain zero
placeholders with `action_valid=false`. A missing action timestamp is represented
as zero and must be interpreted using that validity flag.

### Timing and training validity

The canonical playback clock is `timestamp = frame_index / 10`, with exactly
one frame per original recorded row in each camera video. **This compresses
missed acquisition time; it is not the physical elapsed-time clock.** Original
elapsed seconds remain unchanged in float64 `source_timestamp`.
`frame_gap=true` marks a row whose interval from the previous recorded row
exceeded 0.15 seconds. No images or joint values are interpolated or invented.
Use original timestamps and the raw joined replay for physical elapsed-time analysis.

`training_valid` preserves the recorder's per-row action/camera validity; it does
not certify continuous timing. For temporal/action-chunk training, reject or split
windows crossing any `frame_gap`, and reject invalid rows. Simply filtering out
gap rows and then treating remaining rows as adjacent would still be incorrect.
Use source timestamps if resampling, and keep any synthetic values explicitly invalid.
All-frame statistics describe the archive, including invalid action placeholders;
recompute normalization statistics after choosing training filters.

```python
from datasets import load_dataset
canonical = load_dataset("andlyu/Public-YAM-runs", "lerobot", split="train")
# Camera features are external MP4s declared by meta/info.json, not Parquet images.
```

This targets the v2.1 schema and compatible viewers, not the newer v3 writer.
No LeRobot package or Torch is required by the Jetson publisher.
