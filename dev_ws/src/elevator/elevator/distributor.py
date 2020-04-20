from constants  import *
from util       import *
from elevClass  import Elevator

#~ Calculate the time it takes to get to State_IDLE for one single elevator
def distributor_timeToIdle(elev):
    duration    = 0
    behaviour   = elev.behaviour[elev.id]

    if (behaviour == IDLE):
        elev.direction[elev.id] = util_chooseDirection(elev)
        if (elev.direction[elev.id] == DIRN_STOP):
            return duration
    elif (behaviour == MOVING):
        elev.floor[elev.id] += elev.direction[elev.id]
        duration += TIME_BETWEEN_FLOORS/2
    elif (behaviour == DOOR_OPEN):
        duration -= TIME_DOOR_OPEN/2

    while (1):
        if (util_shouldStop(elev)):
            util_clearAtCurrentFloor(elev)
            duration += TIME_DOOR_OPEN
            elev.direction[elev.id] = util_chooseDirection(elev)
            if (elev.direction[elev.id] == DIRN_STOP):
                return duration
        elev.floor[elev.id] += elev.direction[elev.id]
        duration            += TIME_BETWEEN_FLOORS
    return

def distributor_id(elev, msg):
    min_duration    = 999       # Time duration of elev with least cost
    leastCost_id          = elev.id   # ID of elevator with the least cost

    for id in sorted(elev.queue):
        if (elev.network[id] == ONLINE):
            floor       = elev.floor[id]
            queue       = elev.queue[id]
            dir         = elev.direction[id]
            behaviour   = elev.behaviour[id]
            single_elev = Elevator(id, floor, behaviour, dir, queue)
            single_elev.queue[id][msg.floor][msg.button] = 1
            duration    = distributor_timeToIdle(single_elev)

            if (duration < min_duration):
                min_duration = duration
                leastCost_id = id
    return leastCost_id
