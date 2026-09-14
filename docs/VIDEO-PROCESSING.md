# YAM recordings and Past runs previews

Video processing runs on the robot-side computer in independent workers after
recording finalizes. The browser loads the published MP4; Codex does not encode
or upload it. These workers operate on saved YAM recordings only and never
initialize motors or open cameras. For single and bimanual SO101 recording and exports, see [SO101 recording](SO101-RECORDING.md).

The source under `src/blupe_controller/runtime` includes:

- `YAM_control/training_video.py`: original montage and fast viewing preview.
- `scripts/publish_yam_viewing.py`: small viewing MP4 and public viewing index.
- `scripts/publish_yam_training.py`: full recording/training archive export.
- `scripts/publish_yam_previews.py`: explicit preview backfill tool.

Renderer version 4 skips preview timestamps where an available camera has no
past image within three seconds, including the image's recorded age. This
prevents short `NO RECORDED FRAME` flashes in accelerated Past runs previews.
All panels use the same selected timestamp. The final usable view is retained.
Full-speed archival exports keep explicit gaps and original timing. This does
not detect black pixels inside otherwise valid camera images or fix capture gaps.

Renderer versions participate in video cache and upload receipt checks, so an
older render is not mistaken for a current one just because samples are unchanged.

## Run from a source checkout

Use a separate Python 3.10–3.12 environment and an installed FFmpeg encoder with
libx264. This does not require the motor runtime dependencies:

```sh
python3 -m venv .venv-video
.venv-video/bin/python -m pip install -e '.[video]'
export PYTHONPATH="$PWD/src/blupe_controller/runtime"
export HF_TOKEN_PATH="$HOME/.config/yam-dataset/hf_token"
.venv-video/bin/python -m scripts.publish_yam_viewing \
  --root /path/to/finalized/yam-recordings --keep-published
```

The credential file must already exist with upload access to the intended public
dataset. The default dataset is `andlyu/Public-YAM-runs`; use `--repo` for your
own dataset. Set `YAM_FFMPEG` if the working encoder is outside PATH.

**Keep `--keep-published` for deployments that must not regenerate existing
videos.** Matching successful upload receipts are left unchanged even if their
renderer is older. New/unpublished episodes use the current renderer. Omitting
this flag allows the viewing worker to rebuild old renders whose source frames
are retained. Do not run the backfill tool unless regeneration is intended.

An opt-in Linux user-service example is provided in
[examples/systemd/yam-viewing-publisher.service](../examples/systemd/yam-viewing-publisher.service).
It includes `--keep-published`; adapt its paths before installation. Installation
of the controller package does not enable or restart any publisher service.

To run the independent training worker in the same environment:

```sh
.venv-video/bin/python -m scripts.publish_yam_training \
  --root /path/to/finalized/yam-recordings --repo andlyu/Public-YAM-runs
```

The training worker skips already-uploaded archives. It can reuse a viewing
preview only if its sample hash, file hash and renderer version match.

## September 14 deployment

The matching renderer and publishers were deployed to the existing Jetson with
`--keep-published`. No existing videos were regenerated, including run 312.
The gap around 2.03 seconds in that existing preview remains by request.
Only video publisher services were restarted; no robot run was started.

## Tests

Install pytest in a test environment with the video extra and FFmpeg/FFprobe:

```sh
PYTHONPATH=src:src/blupe_controller/runtime python -m pytest -q \
  tests/test_yam_viewing_first.py tests/test_yam_viewing_preview.py \
  tests/test_yam_video_export.py
```

Tests encode/decode synthetic frames and exercise stale renderer caches,
single-camera dropouts, final-frame retention, publishing retries, and preserving
legacy uploaded receipts with `--keep-published`. They never upload real data.
