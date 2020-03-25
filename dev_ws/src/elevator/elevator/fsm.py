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
driver.elev_init(ELEV_MODE) #Simulator / Physical model
###########################

def fsm_init(e):
    while (driver.elev_get_floor_sensor_signal() == -1):
        driver.elev_set_motor_direction(DIRN_DOWN)

    driver.elev_set_motor_direction(DIRN_STOP)
    e.floor[e.id]       = driver.elev_get_floor_sensor_signal()
    e.behaviour[e.id]   = IDLE
    e.direction[e.id]   = DIRN_STOP
    e.network[e.id]     = ONLINE
    return


#Denne burde kanskje flyttes til et annet sted? Kanskje rename util til utils, og legge til denne der?
def fsm_setAllLights(e): #Call this function when new order is done, as callback, or distributed
    for f in range(N_FLOORS):
        for b in range(N_BUTTONS):
            #for id in id_list: - Implementer dette
                if (e.queue[e.id][f][b]):
                    driver.elev_set_button_lamp(b, f, 1)
                    break
                driver.elev_set_button_lamp(b, f, 0)
    return

def fsm_onInitBetweenFloors(e):
    driver.elev_set_motor_direction(DIRN_DOWN)
    e.direction[e.id] = DIRN_DOWN
    e.behaviour[e.id] = MOVING
    return

def fsm_onNewOrder(e,id,f,b): #When a new order is distributed and confirmed from the cost function
    if (id != e.id):
        e.queue[id][f][b] = 1
        fsm_setAllLights(e)
        return

    bh = e.behaviour[e.id]

    if (bh == DOOR_OPEN):
        if (e.floor[e.id] == f):
            timer_doorsStart()
        else:
            e.queue[e.id][f][b] = 1

    elif (bh == MOVING):
        e.queue[e.id][f][b] = 1

    elif (bh == IDLE):
        if (e.floor[e.id] == f):
            driver.elev_set_door_open_lamp(1)
            timer_doorsStart()
            e.behaviour[e.id] = DOOR_OPEN
        else:
            e.queue[e.id][f][b] = 1
            e.direction[e.id] = util.util_chooseDirection(e)
            driver.elev_set_motor_direction(e.direction[e.id])
            e.behaviour[e.id] = MOVING
            timer_executionStart() # check for mechanical error

    fsm_setAllLights(e)

    return

def fsm_onFloorArrival(e,id,f):
    # other elevator completed
    if (id != e.id):
        for b in range(N_BUTTONS):
            e.queue[id][f][b] = 0
        return

    timer_executionStop() # no mechanical error

    e.floor[e.id] = f
    driver.elev_set_floor_indicator(f)
    if (e.behaviour[e.id] == MOVING):
        if (util.util_shouldStop(e)):
            driver.elev_set_motor_direction(DIRN_STOP)
            driver.elev_set_door_open_lamp(1)
            timer_doorsStart()
            e = util.util_clearAtCurrentFloor(e)
            fsm_setAllLights(e)
            e.behaviour[e.id] = DOOR_OPEN
        else:
            timer_executionStart()

    return

def fsm_onDoorTimeout(e):
    bh = e.behaviour[e.id]
    if (bh == DOOR_OPEN):
        e.direction[e.id] = util.util_chooseDirection(e)
        print(e.direction[e.id])
        driver.elev_set_door_open_lamp(0)
        driver.elev_set_motor_direction(e.direction[e.id])

        if (e.direction[e.id] == DIRN_STOP):
            e.behaviour[e.id] = IDLE
        else:
            e.behaviour[e.id] = MOVING
            timer_executionStart() # check for mechanical error
    return

def fsm_onMechanicalError(e):
    for f in range(N_FLOORS):
        for b in range(N_BUTTONS):
            if (b == BTN_CAB):
                continue
            e.queue[e.id][f][b] = 0

    if (e.floor[e.id] != N_FLOORS-1):
        while (driver.elev_get_floor_sensor_signal() != e.floor[e.id] + 1):
            driver.elev_set_motor_direction(DIRN_UP)
    else:
        while (driver.elev_get_floor_sensor_signal() != e.floor[e.id] - 1):
            driver.elev_set_motor_direction(DIRN_DOWN)

    driver.elev_set_motor_direction(DIRN_STOP)
    driver.elev_set_door_open_lamp(1)
    timer_doorsStart()
    e.floor[e.id]       = driver.elev_get_floor_sensor_signal()
    e.behaviour[e.id]   = DOOR_OPEN
    e.direction[e.id]   = DIRN_STOP
    e.network[e.id]     = ONLINE
    e = util.util_clearAtCurrentFloor(e)
    return
