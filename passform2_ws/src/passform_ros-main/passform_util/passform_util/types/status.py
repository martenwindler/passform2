from enum import IntEnum

class Status(IntEnum):
    OK = 0
    WARN = 1
    ERROR = 2
    STALE = 3
    RUNNING = 4
    
class ConnectionStatus(IntEnum):
    CONNECTED = 0
    DISCONNECTED = 1
    TIMEOUT = 2
    UNKNOWN_ERROR = 3
