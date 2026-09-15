# SO101 position-read retries

The single SO101 adapter configures its own LeRobot bus to use `num_retry=3`
for `Present_Position` sync reads: one initial packet attempt plus at most three
retries. The SDK stops retrying as soon as a packet succeeds. This covers both
feedback reads and the pre-command position check used by relative-target limits.
No motion write, torque operation, or complete movement is replayed.

If all four packet attempts fail, the existing error propagates and latches the
controller fault. Invalid feedback and calibration/limit failures still fail
immediately; successful later reads do not clear a latched fault. Retries use the
existing SDK serial timeout and add no deliberate sleep. As with other blocking
servo IO, their bounded attempts can delay a waiting watchdog's acquisition of
the driver lock. No runtime duration or safety limit is changed.
