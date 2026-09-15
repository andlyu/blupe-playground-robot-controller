# SO101 position-read retries

The single SO101 adapter configures its own LeRobot bus to perform
one initial `Present_Position` sync read plus at most three retries, waiting
0.5 seconds before each retry. SDK-internal retries are disabled so attempts
are not multiplied. A successful read returns immediately. This covers both
feedback reads and the pre-command position check used by relative-target limits.
No motion write, torque operation, or complete movement is replayed.

If all four packet attempts fail, the existing error propagates and latches the
controller fault. Invalid feedback and calibration/limit failures still fail
immediately; successful later reads do not clear a latched fault. Each attempt uses the
existing SDK serial timeout; three retries add at most 1.5 seconds of deliberate
waiting, in addition to serial IO time. As with other blocking
servo IO, their bounded attempts can delay a waiting watchdog's acquisition of
the driver lock. No runtime duration or safety limit is changed.
