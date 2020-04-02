from ctypes import *
import os
import status
import sys
import util

from constants import *
from timer import *

install_dir = os.path.join(os.path.dirname(__file__))
sys.path.append(os.path.abspath(install_dir))

###### LOAD DRIVER ########
driver_path = os.path.join(os.path.dirname(__file__), '../../../../../../src/elevator/elevator/driver/driver.so')
driver = cdll.LoadLibrary(os.path.abspath(driver_path))
#driver.elev_init(ELEV_MODE) #Simulator / Physical model
###########################

def fsm_init(elev):
    while (driver.elev_get_floor_sensor_signal() == -1):
        driver.elev_set_motor_direction(DIRN_DOWN)

    driver.elev_set_motor_direction(DIRN_STOP)
    elev.floor[elev.id]       = driver.elev_get_floor_sensor_signal()
    elev.behaviour[elev.id]   = IDLE
    elev.direction[elev.id]   = DIRN_STOP
    elev.network[elev.id]     = ONLINE

    driver.elev_set_floor_indicator(elev.floor[elev.id])
    util.util_setAllLights(elev)
    return

def fsm_onInitBetweenFloors(elev):

    driver.elev_set_motor_direction(DIRN_DOWN)
    elev.direction[elev.id] = DIRN_DOWN
    elev.behaviour[elev.id] = MOVING
    return

def fsm_onNewOrder(elev, id, floor, btn): #When a new order is distributed and confirmed from the cost function
    if (id != elev.id):
        elev.queue[id][floor][btn] = 1
        util.util_setAllLights(elev)
        return

    bh = elev.behaviour[elev.id]

    if (bh == DOOR_OPEN):
        if (elev.floor[elev.id] == floor):
            timer_doorsStart()
        else:
            elev.queue[elev.id][floor][btn] = 1

    elif (bh == MOVING):
        elev.queue[elev.id][floor][btn] = 1

    elif (bh == IDLE):
        if (elev.floor[elev.id] == floor):
            driver.elev_set_door_open_lamp(1)
            timer_doorsStart()
            elev.behaviour[elev.id] = DOOR_OPEN
        else:
            elev.queue[elev.id][floor][btn] = 1
            elev.direction[elev.id] = util.util_chooseDirection(elev)
            driver.elev_set_motor_direction(elev.direction[elev.id])
            elev.behaviour[elev.id] = MOVING
            timer_executionStart() # check for mechanical error

    util.util_setAllLights(elev)

    return

def fsm_onFloorArrival(elev, id, floor):
    # other elevator completed
    if (id != elev.id):
        for b in range(N_BUTTONS):
            elev.queue[id][floor][b] = 0
        util.util_setAllLights(elev)
        return

    timer_executionStop() # no mechanical error

    elev.floor[elev.id] = floor
    driver.elev_set_floor_indicator(floor)
    if (elev.behaviour[elev.id] == MOVING):
        if (util.util_shouldStop(elev)):
            driver.elev_set_motor_direction(DIRN_STOP)
            driver.elev_set_door_open_lamp(1)
            timer_doorsStart()
            util.util_clearAtCurrentFloor(elev)
            util.util_setAllLights(elev)
            elev.behaviour[elev.id] = DOOR_OPEN
        else:
            timer_executionStart()

    return

def fsm_onDoorTimeout(elev):
    bh = elev.behaviour[elev.id]
    if (bh == DOOR_OPEN):
        elev.direction[elev.id] = util.util_chooseDirection(elev)
        driver.elev_set_door_open_lamp(0)
        driver.elev_set_motor_direction(elev.direction[elev.id])

        if (elev.direction[elev.id] == DIRN_STOP):
            elev.behaviour[elev.id] = IDLE
        else:
            elev.behaviour[elev.id] = MOVING
            timer_executionStart() # check for mechanical error
    return

def fsm_onMechanicalError(elev):
    for f in range(N_FLOORS):
        for b in range(N_BUTTONS):
            if (b == BTN_CAB):
                continue
            elev.queue[elev.id][f][b] = 0

    if (elev.floor != N_FLOORS-1):
        while (driver.elev_get_floor_sensor_signal() != elev.floor[elev.id] + 1):
            driver.elev_set_motor_direction(DIRN_UP)
    else:
        while (driver.elev_get_floor_sensor_signal() != elev.floor[elev.id] - 1):
            driver.elev_set_motor_direction(DIRN_DOWN)

    driver.elev_set_motor_direction(DIRN_STOP)
    driver.elev_set_door_open_lamp(1)
    timer_doorsStart()
    elev.floor[elev.id]       = driver.elev_get_floor_sensor_signal()
    elev.behaviour[elev.id]   = DOOR_OPEN
    elev.direction[elev.id]   = DIRN_STOP
    elev.network[elev.id]     = ONLINE
    return
