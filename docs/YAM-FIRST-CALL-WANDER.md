# First-call wandering (YAM only)

The YAM controller can perform a small outward wandering motion while the first
model request is pending. Enable with `YAM_FIRST_CALL_WANDER=1` on the controller.
Installation alone leaves it disabled. The runner must use the authenticated
Session API executor described below; no motion starts merely because a session
is prepared or a public conversation is published.

The controller plans from measured joints, preserves tool orientation and gripper
positions, and checks joint limits, speed, workspace and swept collisions for the
whole round trip. It tries smaller Cartesian paths if the full preview cannot
pass. Closed or partly closed grippers skip the animation. The collision model
cannot detect arbitrary people or newly placed objects: operate only with the
outward sweep clear. Open fingers are not proof that the workspace is clear.

It starts only before the first policy command and is consumed once per lease.
After at most ten seconds, or when `finish` arrives, it retraces the path and waits
for measured settling within 0.02 radians. Return can take another ten seconds
plus settling; it is never sped up to match the illustrative animation. Stop,
lease loss and faults cancel motion without an automatic return. An uncertain
return blocks policy commands; there is no automatic movement retry.

## Runner contract

Use existing session-capability-authenticated `/v1/sessions/{id}/executions`:
`executor: "yam_first_call"`, operations `start`, `finish`, `status`, empty
`payload: {}`, plus the usual schema version, request ID, episode and lease IDs.
The Session API must explicitly support this executor without consuming policy
step IDs or marking the session as a spatial-MPC-only session.

Send `start` **after the first camera images have been captured**, immediately
before the first inference request. Send `finish` in inference cleanup, including
model errors. Await its completed result with `state: "returned"` or `"skipped"`
and `settled: true` before interpreting/executing the model's first action.
Subsequent model calls do not send these operations. Failed/ambiguous writes
stop the run rather than retrying motion. Changed scene content requires fresh
images and replanning. This is an execution handshake, not a display event.

`blupe-controller ... run` installs the YAM integration automatically. A custom
YAM operator entry point can call
`YAM_control.first_call_wander_integration.install(hardware_module)` before
constructing its operator; it preserves other executor handlers and the existing
hardware safety configuration. No SO101 behavior changes.

Hardware-free tests:

```
python -m unittest discover -s tests -p test_first_call_wander.py
```

The preview's roughly 49 cm excursion is not a hardware guarantee. On the tested
Robo-house saved home/configuration, the planner accepted a 0.4 distance scale
(about 20 cm per grasp point). Every live start repeats planning and validation
from that rig's actual measured pose.
