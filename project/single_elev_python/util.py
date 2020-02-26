import constants
import status


def util_orderAbove(e): #Ta inn status klassen til seg selv
    for f in range(e.floor + 1, constants.N_FLOORS):
        for b in range(constants.N_BUTTONS):
            if e.queue[e.id][f][b]:
                return 1
    return 0

def util_orderBelow(e): #Ta inn status klassen til seg selv
    for f in range(0, e.floor):
        for b in range(constants.N_BUTTONS):
            if e.queue[e.id][f][b]:
                return 1
    return 0

def util_chooseDirection(e):
    dir = e.direction
    if (dir == constants.DIRN_UP):
        if (util_orderAbove(e)):
            return constants.DIRN_UP
        elif (util_orderBelow(e)):
            return constants.DIRN_DOWN
        else:
            return constants.DIRN_STOP
    elif ((dir == constants.DIRN_DOWN) or (dir == constants.DIRN_STOP)):
        if (util_orderBelow(e)):
            return constants.DIRN_DOWN
        elif (util_orderAbove(e)):
            return constants.DIRN_UP
        else:
            return constants.DIRN_STOP
    else:
        return constants.DIRN_STOP

def util_shouldStop(e):
    dir = e.direction
    if (dir == constants.DIRN_DOWN):
        return ((e.queue[e.id][e.floor][constants.BTN_HALL_DOWN]) or
                (e.queue[e.id][e.floor][constants.BTN_CAB]) or
                not(util_orderBelow(e)))
    elif (dir == constants.DIRN_UP):
        return ((e.queue[e.id][e.floor][constants.BTN_HALL_UP]) or
                (e.queue[e.id][e.floor][constants.BTN_CAB]) or
                not(util_orderAbove(e)))
    else:
        return 1

def util_clearAtCurrentFloor(e):
    for b in range(constants.N_BUTTONS):
        if (e.queue[e.id][e.floor][b]):
            e.queue[e.id][e.floor][b] = 0
    return e
