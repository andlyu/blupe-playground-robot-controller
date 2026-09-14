# SO101 recording and automatic uploads

This main-branch source supports single SO101 and bimanual SO101 recording.
Pulling the code does not enable recording or cloud uploads by itself. Existing
calibration, serial ports, poses and camera assignments should remain in place.

## On the Mac running the controller

1. Wait until the controller reports no active session, command or manual motion.
   Pause auto-queue before stopping/restarting the existing controller process.
2. In this checkout, run `git pull --ff-only origin main`. If it reports local
   modifications or diverged history, preserve that work and resolve it before
   continuing. Do not use a hard reset.
3. In the controller's existing Python 3.10–3.12 environment, run
   `python -m pip install -e . --no-deps`. Use the environment already running the
   arm; the recording module adds no camera or motor dependencies.
4. Create a private recording directory, for example:

   ```sh
   mkdir -p "$HOME/blupe-recordings"
   chmod 700 "$HOME/blupe-recordings"
   ```

5. Back up the JSON profile passed to `blupe-controller --config`. Add an absolute
   path under its existing settings (substitute the actual Mac login name):

   ```json
   {"settings": {"recording_root": "/Users/YOUR_LOGIN/blupe-recordings"}}
   ```

   Merge this setting; do not replace the rest of the profile. `~` is not expanded.
   Keep `hardware: "so101"`, the registered robot ID, calibration and named camera
   mapping. Capture uses the existing relay at `settings.camera_port` (default
   8089), requesting `/<device>/snapshot.jpg`; do not launch a second camera owner.
6. Restart the existing controller with that same profile. Verify its operator
   status, camera freshness and `cloud_execution.recording_error`. Restore queue
   readiness only after checking the physical arm and the normal Home conditions.

An accepted cloud session automatically starts `ep_<id>/manifest.json`,
`samples.jsonl` and `images/`. Stop, disconnect and controller errors finalize it.
An idle controller creates no episode. Verify a subsequent operator-authorized
run has `status: "finalized"`, nonzero rows and camera frames. This setup does not
require a calibration procedure or a test motion.

## Cloud publisher setup (required separately)

The existing cloud publisher can pull finalized episodes from the Mac using
rsync over SSH. Enable macOS Remote Login only for the intended account, install
a dedicated read-only restricted rsync key for the recording directory, and pin
its host key. If inbound access is unavailable, provision a dedicated reverse
SSH tunnel on an approved unused port. The operator-console tunnel alone does
not provide recording-file access. Never commit SSH keys or HF tokens.

Use the shipped publisher from a source checkout on the upload host:

```sh
python3 -m venv .venv-video
.venv-video/bin/python -m pip install -e '.[video]'
export PYTHONPATH="$PWD/src/blupe_controller/runtime"
export HF_TOKEN_PATH="$HOME/.config/blupe-dataset/hf_token"
.venv-video/bin/python -m scripts.sync_robot_recordings \
  --root /path/to/cloud-cache/so101 \
  --host MAC_USER@MAC_HOST \
  --ssh 'ssh -i /path/to/read-only-key -o IdentitiesOnly=yes -o BatchMode=yes -o StrictHostKeyChecking=yes' \
  --repo andlyu/Public-YAM-runs \
  --visualizer-repo OWNER/APPROVED_SO101_VISUALIZER_REPO
```

Install FFmpeg with libx264 on the upload host. Provision the approved visualizer
repository and HF upload credential before enabling this command as a persistent
service; it does not create repositories. The rsync account must be restricted
so its remote `./` is the recording root. Keep credentials on the upload host.

The raw archive retains a separate native-schema subset per robot. Past runs
mixes all robots. The LeRobot visualizer needs its own robot-specific repository
root with camera videos and joint traces; add the approved mapping in the public
runner's `codex-runner/static/hosted.js`. Do not point a new arm at another robot's
visualizer. The Mac SO101 destination is not preconfigured in this release.

Both publisher receipts must succeed before cloud working copies are removed.
Robot originals remain on the Mac. A recording without a visualizer receipt is
backfilled automatically. Synthetic/simulated episodes are never published.

MakerMods currently uses `andlyu/Public-MakerMods-SO101-runs` for its visualizer
and `andlyu/Public-YAM-runs` for the shared raw archive. That deployment does not
automatically configure the single SO101 Mac.
