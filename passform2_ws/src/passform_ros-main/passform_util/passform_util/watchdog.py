from threading import Timer
from typing import Callable

# taken from https://newbedev.com/how-to-implement-a-watchdog-timer-in-python
class Watchdog(Exception):
    def __init__(self, timeout, callback: Callable):  # timeout in seconds
        self.timeout = timeout
        self.callback = callback
        self.timer = Timer(self.timeout, self.callback)
        self.timer.start()

    @property
    def callback(self):
        return self._callback

    @callback.setter
    def callback(self, value):
        self._callback = value

    def reset(self):
        self.timer.cancel()
        self.timer = Timer(self.timeout, self.callback)
        self.timer.start()

    def stop(self):
        self.timer.cancel()

    def defaultHandler(self):
        raise self

if __name__ == '__main__':
    from time import sleep
    wd = Watchdog(3)
    sleep(5)
