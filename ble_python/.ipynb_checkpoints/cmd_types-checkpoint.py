from enum import Enum

class CMD(Enum):
    PING = 0
    SEND_TWO_INTS = 1
    GET_TOF_5s = 2
    GET_TOF_IMU = 3
    START = 4
    STOP = 5
    START_PID = 6
    START_MAP = 7
    START_STUNT = 8
