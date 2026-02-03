from enum import Enum


class CMD(Enum):
    PING = 0
    SEND_TWO_INTS = 1
    SEND_THREE_FLOATS = 2
    GET_TIME_MILLIS = 3
    ECHO = 4
    DANCE = 5
    SET_VEL = 6
    STORE_TIME_MILLIS = 7
    SEND_TIME_MILLIS = 8
