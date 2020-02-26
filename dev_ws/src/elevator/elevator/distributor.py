import fsm


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

def distributor_timeToIdle(e): #Calculates the time it takes to get to IDLE
    duration = 0
    bh = e.behaviour

    if (bh == IDLE):
        e.direction = util_chooseDirection(e)
        if (e.direction == DIRN_STOP):
            return duration
    elif (bh == MOVING):
        duration += TIME_BETWEEN_FLOORS/2
        e.floor += e.direction
    elif (bh == DOOR_OPEN):
        duration -= TIME_DOOR_OPEN/2
    while (1):
        if (util_shouldStop(e)):
            e = util_clearAtCurrentFloor(e)
            duration += TIME_DOOR_OPEN
            e.direction = util_chooseDirection(e)
            if (e.direction == DIRN_STOP):
                return duration

        e.floor += e.direction
        duration += TIME_BETWEEN_FLOORS
