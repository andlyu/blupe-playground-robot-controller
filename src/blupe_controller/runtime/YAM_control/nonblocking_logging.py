"""Best-effort worker logs: producers never wait for a queue lock or output sink.

Only the daemon writer formats records and touches the original stream. Log
loss is counted; safety state and the motor trace remain independent of logging.
This removes synchronous I/O/handler lock waits, not Python scheduling latency.
"""
from collections import deque
import logging
import sys
import threading


class BufferedLogs:
    def __init__(self, sink, capacity=1024):
        if capacity < 1:
            raise ValueError('capacity must be positive')
        self.sink = sink
        self.capacity = capacity
        self.items = deque()
        self.lock = threading.Lock()
        self.dropped = 0
        self.sink_errors = 0
        self.written = 0
        self.stopped = threading.Event()
        self.formatter = logging.Formatter('%(levelname)s:%(name)s:%(message)s')
        self.thread = threading.Thread(target=self._run, name='motor-log-writer', daemon=True)
        self.thread.start()

    def offer(self, item):
        if not self.lock.acquire(blocking=False):
            self.dropped += 1
            return
        try:
            if len(self.items) >= self.capacity:
                self.dropped += 1
            else:
                self.items.append(item)
        finally:
            self.lock.release()

    def stats(self):
        return dict(dropped=self.dropped, sink_errors=self.sink_errors,
                    written=self.written, queued=len(self.items), capacity=self.capacity)

    def _run(self):
        while not self.stopped.is_set():
            with self.lock:
                item = self.items.popleft() if self.items else None
            if item is None:
                self.stopped.wait(.01)
                continue
            try:
                text = self.formatter.format(item) + '\n' if isinstance(item, logging.LogRecord) else item
                self.sink.write(text)
                self.sink.flush()
                self.written += 1
            except Exception:
                # Never recurse through logging when the logging sink fails.
                self.sink_errors += 1


class BufferedHandler(logging.Handler):
    def __init__(self, buffer):
        super().__init__()
        self.buffer = buffer

    def handle(self, record):
        # Handler.handle normally takes a shared lock before emit.
        self.buffer.offer(record)
        return True

    def emit(self, record):
        self.buffer.offer(record)


class BufferedStream:
    encoding = 'utf-8'
    errors = 'replace'

    def __init__(self, buffer):
        self.buffer = buffer

    def write(self, text):
        # print/traceback call write with strings; bound each retained chunk.
        if text:
            self.buffer.offer(text[:8192])
            if len(text) > 8192:
                self.buffer.dropped += 1
        return len(text)

    def flush(self):
        pass  # flush=True must not wait for the output sink.

    def isatty(self):
        return False


def install():
    """Call once in the fresh motor process, before importing hardware drivers."""
    buffer = BufferedLogs(sys.stderr)
    handler = BufferedHandler(buffer)
    root = logging.getLogger()
    root.handlers[:] = [handler]
    root.setLevel(logging.INFO)
    # Existing named handlers must not bypass the nonblocking root handler.
    for logger in logging.Logger.manager.loggerDict.values():
        if isinstance(logger, logging.Logger):
            logger.handlers.clear()
            logger.propagate = True
    sys.stdout = BufferedStream(buffer)
    sys.stderr = BufferedStream(buffer)
    return buffer
