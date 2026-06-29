import threading


class ReadWriteLock:
    def __init__(self):
        self._condition = threading.Condition(threading.Lock())
        self._readers = 0

    def acquire_read(self):
        with self._condition:
            self._condition.wait_for(lambda: self._readers >= 0)
            self._readers += 1

    def release_read(self):
        with self._condition:
            self._readers -= 1
            if self._readers == 0:
                self._condition.notify_all()

    def acquire_write(self):
        with self._condition:
            self._condition.wait_for(lambda: self._readers == 0)
            self._readers = -1

    def release_write(self):
        with self._condition:
            self._readers = 0
            self._condition.notify_all()
