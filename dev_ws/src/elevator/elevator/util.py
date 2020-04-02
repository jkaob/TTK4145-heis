import status
from constants import *


def util_orderAbove(elev): #Ta inn status klassen til seg selv
    for f in range(elev.floor[elev.id] + 1, N_FLOORS):
        for b in range(N_BUTTONS):
            if (elev.queue[elev.id][f][b]):
                return 1
    return 0

def util_orderBelow(elev): #Ta inn status klassen til seg selv
    for f in range(0, elev.floor[elev.id]):
        for b in range(N_BUTTONS):
            if (elev.queue[elev.id][f][b]):
                return 1
    return 0

def util_chooseDirection(elev):
    dir = elev.direction[elev.id]
    if (dir == DIRN_UP):
        if (util_orderAbove(elev)):
            return DIRN_UP
        elif (util_orderBelow(elev)):
            return DIRN_DOWN
        else:
            return DIRN_STOP

    elif ((dir == DIRN_DOWN) or (dir == DIRN_STOP)):
        if (util_orderBelow(elev)):
            return DIRN_DOWN
        elif (util_orderAbove(elev)):
            return DIRN_UP
        else:
            return DIRN_STOP

    else:
        return DIRN_STOP

def util_shouldStop(elev):
    dir = elev.direction[elev.id]
    if (dir == DIRN_DOWN):
        return ((elev.queue[elev.id][elev.floor[elev.id]][BTN_HALL_DOWN]) or
                (elev.queue[elev.id][elev.floor[elev.id]][BTN_CAB]) or
                not(util_orderBelow(elev)))
    elif (dir == DIRN_UP):
        return ((elev.queue[elev.id][elev.floor[elev.id]][BTN_HALL_UP]) or
                (elev.queue[elev.id][elev.floor[elev.id]][BTN_CAB]) or
                not(util_orderAbove(elev)))
    else:
        return 1

def util_clearAtCurrentFloor(elev):
    for b in range(N_BUTTONS):
        if (elev.queue[elev.id][elev.floor[elev.id]][b]):
            elev.queue[elev.id][elev.floor[elev.id]][b] = 0
    return
