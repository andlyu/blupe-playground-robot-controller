# Reset bin through the normal Session API lease

Install `YAM_control.cube_session_executor.install(hardware)` around the existing
operator class. It adds the `yam_cube_reset` executor to the existing authenticated
Session API transport. It never opens CAN or constructs a second motor adapter.

The operator enables this executor with `YAM_CUBE_RESET_SESSION_ENABLED=1`.
`YAM_CUBE_RESET_EXECUTION_POLICY` defaults to `validated`; the explicitly selected
`controller-faults` profile retains motor bounds, cadence/size, step/velocity,
health/tracking and Stop while omitting geometry predictions, matching the cube
program's existing selected mode. Other Session API trajectories keep their own
existing checks. Calibration, gains, Home and Park are unchanged.

`preview`, `execute`, `wait` and `status` require the current authorized session,
episode and lease, plus an unexpired coordinator envelope. A preview is used once.
Native pending-command admission prevents overlap with ordinary packets. Poses
remain in `API_ACTIVE` and retain the original lease while the existing motor
adapter executes. No local-pose reservation or queue pause is acquired.

Stop/connection loss/runtime expiry use the existing native Hold/Stop path and
cancel the executor's dispatch event. A late worker cannot restore mode, authority
or pending state. Settling warnings under `controller-faults` retain the existing
cube semantics; native motor faults remain terminal. No movement is retried.

The optional 0.1/0.8-second floors require the exact previously qualified native
linear helper and forwarding wrapper hashes. Capability status only advertises
that profile when the installed helper matches. Fresh measured gripper feedback
retains the existing 0.1 normalized-step interpolation.

Deployment must preserve the live adapter and unrelated wrappers, source-check
and back up changes, then activate only with fresh verified torque-off and no
active run. Check `cube_reset_session` status after activation. Offline tests and
inert coordinator/adapter tests do not establish a physical reset completion.
# Recorded Replay under the same lease

Preview accepts `motion_kind: recorded_replay` with the configured saved recipe
and one `replay_request_id`. It reuses the installed PrimaryReplay loader and
qualified engine (source/calibration/edit checks and its existing motor guards).
It does not call manual Replay admission or switch API_ACTIVE to REPLAYING.
The engine's guard additionally checks the current session lease at each approval;
normal Hold/Stop cancels its existing event. Completion waits for the existing
engine thread to release its physical lock and requires the matching full
complete/Hold receipt. A late finish cannot restore a revoked lease.

`motion_kind: camera_clear_home` uses the native `home` target and current grippers
through the existing bounded pose executor. It does not invoke Park, initialization
or torque-off. The caller verifies measured Home before any tabletop sensing.
Ordinary cube pose requests and default validation behavior remain unchanged.
