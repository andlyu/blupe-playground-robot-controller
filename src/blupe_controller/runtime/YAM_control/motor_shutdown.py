"""Verified i2rt shutdown: join BOTH producer threads before sending motor-off."""


def disable_motorchain(robot):
    chain = robot.motor_chain
    try:
        count = len(chain)
    except Exception:
        count = 7
    result = dict(ok=False, motor_count=count, disabled_motor_ids=[], errors=[])
    def fail(stage, exc, **context):
        result['errors'].append(dict(stage=stage, message=str(exc), **context))
    try:
        robot._stop_event.set()
        robot._server_thread.join(timeout=2.0)
        if robot._server_thread.is_alive():
            raise RuntimeError('robot control thread did not stop')
        # Requires the pinned i2rt stop_thread patch. A boolean is not a join:
        # the CAN worker may be inside a transaction or motor recovery.
        chain.stop_thread(timeout=2.0)
    except Exception as exc:
        fail('controller_shutdown', exc)
        return result  # Never race motor_off/close against an active CAN worker.
    for mid in range(1, count + 1):
        try:
            chain.motor_interface.motor_off(mid)
            result['disabled_motor_ids'].append(mid)
        except Exception as exc:
            fail('motor_off', exc, motor_id=mid)
    if not result['errors']:
        try:
            chain.close()
        except Exception as exc:
            fail('can_close', exc)
    # Retain the stopped-but-open bus on failure so a verified retry is possible.
    result['ok'] = not result['errors']
    return result
