"""Bounded retries for an overloaded position RPC, never for motor faults."""
import time


class PositionFetchBusy(RuntimeError):
    pass


class PositionFetchCancelled(RuntimeError):
    pass


def fetch_position(snapshot, authorized, *, sleep=time.sleep, on_retry=None):
    """Initial request plus at most three retries, each 500 ms apart."""
    for attempt in range(4):
        if not authorized():
            raise PositionFetchCancelled('Authority revoked while fetching joint positions')
        try:
            result = snapshot()
        except RuntimeError as exc:
            if str(exc) != 'Motor operation capacity exceeded':
                raise
            if attempt == 3:
                raise PositionFetchBusy(
                    'Unable to obtain current joint positions: motor worker busy after '
                    '4 requests (3 retries, 500 ms apart)') from exc
            if on_retry is not None:
                on_retry(attempt + 1)
            sleep(.5)
        else:
            if not authorized():
                raise PositionFetchCancelled('Authority revoked while fetching joint positions')
            return result
