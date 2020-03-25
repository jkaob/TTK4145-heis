import status
from constants import *


def util_orderAbove(e): #Ta inn status klassen til seg selv
    for f in range(e.floor[e.id] + 1, N_FLOORS):
        for b in range(N_BUTTONS):
            if (e.queue[e.id][f][b]):
                return 1
    return 0

def util_orderBelow(e): #Ta inn status klassen til seg selv
    for f in range(0, e.floor[e.id]):
        for b in range(N_BUTTONS):
            if (e.queue[e.id][f][b]):
                return 1
    return 0

def util_chooseDirection(e):
    dir = e.direction[e.id]
    if (dir == DIRN_UP):
        if (util_orderAbove(e)):
            return DIRN_UP
        elif (util_orderBelow(e)):
            return DIRN_DOWN
        else:
            return DIRN_STOP

    elif ((dir == DIRN_DOWN) or (dir == DIRN_STOP)):
        if (util_orderBelow(e)):
            return DIRN_DOWN
        elif (util_orderAbove(e)):
            return DIRN_UP
        else:
            return DIRN_STOP

    else:
        return DIRN_STOP

def util_shouldStop(e):
    dir = e.direction[e.id]
    if (dir == DIRN_DOWN):
        return ((e.queue[e.id][e.floor[e.id]][BTN_HALL_DOWN]) or
                (e.queue[e.id][e.floor[e.id]][BTN_CAB]) or
                not(util_orderBelow(e)))
    elif (dir == DIRN_UP):
        return ((e.queue[e.id][e.floor[e.id]][BTN_HALL_UP]) or
                (e.queue[e.id][e.floor[e.id]][BTN_CAB]) or
                not(util_orderAbove(e)))
    else:
        return 1

def util_clearAtCurrentFloor(e):
    for b in range(N_BUTTONS):
        if (e.queue[e.id][e.floor[e.id]][b]):
            e.queue[e.id][e.floor[e.id]][b] = 0
    return e
