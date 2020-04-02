import fsm
from constants import *
import util
# How to use this functions
# When a new order is received via ROS or from your own buttons
# We must:
# 0. duration = 0
# 1. For elevators:
    #2. elevator copy = elevaor
    #3. temp duration = distributor_timeToIdle(elevator copy)
    #4. if temp duration <= duration
        #id = elevator copy id
        #duration = temp duration
#We should now have the id of the elevator that has the least time to serve that order
#Assign the new order to the corresponding id

def distributor_timeToIdle(elev): #Calculates the time it takes to get to IDLE
    duration = 0
    behaviour = elev.behaviour[elev.id]

    if (behaviour == IDLE):
        elev.direction[elev.id] = util.util_chooseDirection(elev)
        if (elev.direction[elev.id] == DIRN_STOP):
            return duration

    elif (behaviour == MOVING):
        duration += TIME_BETWEEN_FLOORS/2
        elev.floor[elev.id] += elev.direction[elev.id]

    elif (behaviour == DOOR_OPEN):
        duration -= TIME_DOOR_OPEN/2

    while (1):
        if (util.util_shouldStop(elev)):
            util.util_clearAtCurrentFloor(elev)
            duration += TIME_DOOR_OPEN
            elev.direction[elev.id] = util.util_chooseDirection(elev)
            if (elev.direction[elev.id] == DIRN_STOP):
                return duration

        elev.floor[elev.id] += elev.direction[elev.id]
        duration += TIME_BETWEEN_FLOORS

    return
