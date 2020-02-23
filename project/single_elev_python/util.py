import constants
import status


def util_orderAbove(e): #Ta inn status klassen til seg selv
    for f in range(e.floor + 1, N_FLOORS):
        for b in range(N_BUTTONS):
            if e.queue[e.id][f][b]:
                return 1
    return 0

def util_orderBelow(e): #Ta inn status klassen til seg selv
    for f in range(0, e.floor):
        for b in range(N_BUTTONS):
            if e.queue[e.id][f][b]:
                return 1
    return 0

def util_chooseDirection(e):
    dir = e.direction
    if (dir == DIRN_UP):
        if (util_orderAbove(e)):
            return DIRN_UP
        else if (util_orderBelow(e)):
            return DIRN_DOWN
        else:
            return DIRN_STOP
    else if ((dir == DIRN_DOWN) || (dir == DIRN_STOP)):
        if (util_orderBelow(e)):
            return DIRN_DOWN
        else if (util_orderAbove(e)):
            return DIRN_UP
        else:
            return DIRN_STOP
    else:
        return DIRN_STOP

def util_shouldStop(e):
    dir = e.direction
    if (dir == DIRN_DOWN):
        return ((e.queue[e.id][e.floor][BTN_HALL_DOWN]) or
                (e.queue[e.id][e.floor][BTN_CAB]) or
                not(util_orderBelow(e)))
    else if (dir == DIRN_UP):
        return ((e.queue[e.id][e.floor][BTN_HALL_UP]) or
                (e.queue[e.id][e.floor][BTN_CAB]) or
                not(util_orderAbove(e)))
    else:
        return 1

def util_clearAtCurrentFloor(e):
    for (b in range(N_BUTTONS)):
        if (e.queue[e.id][e.floor][b]):
            e.queue[e.id][e.floor][b] = 0
    return e
