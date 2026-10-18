# Operator Park execution profile

Park uses the calibrated `park_pose` when configured, preserves its exact endpoint, verifies measured arrival, then requests torque-off and marks the queue parked only after verified motor-off. A rejected preview or missed settling retains a healthy Hold; explicit Stop and native motor fault handling remain independent.

Default `YAM_PARK_EXECUTION_POLICY=validated` retains geometry validation. An operator-selected station already using the `controller-faults` profile may set `YAM_PARK_EXECUTION_POLICY=controller-faults` to use the same motor-guard mode for Park. That profile omits Cartesian/collision-model queries while retaining finite12-joint targets, selected joint/start limits, trajectory size,10Hz cadence, adjacent-step and velocity limits, native motor health/tracking, cancellation and measured settling. Do not select it silently for other stations. Model-predicted contact and physical contact are different evidence.

Activate controller code only while the existing owner is idle and torque-off verified. Configuration changes do not authorize a physical Park or queue start. Offline tests do not establish physical completion.
