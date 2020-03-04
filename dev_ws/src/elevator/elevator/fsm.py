import sys
import os
install_dir = os.path.join(os.path.dirname(__file__))
sys.path.append(os.path.abspath(install_dir))

from ctypes import *
import constants
import status
import util
import timer
import sys


###### LOAD DRIVER ########
driver_path = os.path.join(os.path.dirname(__file__), '../../../../../../src/elevator/elevator/driver/driver.so')
driver = cdll.LoadLibrary(os.path.abspath(driver_path))
driver.elev_init(0)
###########################

def fsm_init(e):
    while(driver.elev_get_floor_sensor_signal() == -1):
        driver.elev_set_motor_direction(constants.DIRN_DOWN)
    driver.elev_set_motor_direction(constants.DIRN_STOP)
    e.floor[e.id] = driver.elev_get_floor_sensor_signal()
    e.behaviour[e.id] = constants.IDLE
    e.direction[e.id] = constants.DIRN_STOP
    return


#Denne burde kanskje flyttes til et annet sted? Kanskje rename util til utils, og legge til denne der?
def fsm_setAllLights(e): #Call this function when new order is done, as callback, or distributed
    for f in range(constants.N_FLOORS):
        for b in range(constants.N_BUTTONS):
            #for id in id_list: - Implementer dette
                if (e.queue[e.id][f][b]):
                    driver.elev_set_button_lamp(b, f, 1)
                    break
                driver.elev_set_button_lamp(b, f, 0)
    return

def fsm_onInitBetweenFloors(e):
    driver.elev_set_motor_direction(DIRN_DOWN)
    e.direction[e.id] =DIRN_DOWN
    e.behaviour[e.id] = MOVING
    return

def fsm_onNewOrder(e,id,f,b): #When a new order is distributed and confirmed from the cost function
    if(id != e.id):
        e.queue[id][f][b] = 1
        return

    bh = e.behaviour[e.id]

    if (bh == constants.DOOR_OPEN):
        if (e.floor[e.id] == f):
            timer.timer_start()
        else:
            e.queue[e.id][f][b] = 1

    elif (bh == constants.MOVING):
        e.queue[e.id][f][b] = 1

    elif (bh == constants.IDLE):
        if (e.floor[e.id] == f):
            driver.elev_set_door_open_lamp(1)
            timer.timer_start()
            e.behaviour[e.id] = constants.DOOR_OPEN
        else:
            e.queue[e.id][f][b] = 1
            e.direction[e.id] = util.util_chooseDirection(e)
            driver.elev_set_motor_direction(e.direction[e.id])
            e.behaviour[e.id] = constants.MOVING

    fsm_setAllLights(e)
    return

def fsm_onFloorArrival(e,id,f):
    if(id != e.id):
        for b in range(constants.N_BUTTONS):
            e.queue[id][f][b] = 0
        return

    e.floor[e.id] = f
    driver.elev_set_floor_indicator(f)
    if (e.behaviour[e.id] == constants.MOVING):
        if (util.util_shouldStop(e)):
            driver.elev_set_motor_direction(constants.DIRN_STOP)
            driver.elev_set_door_open_lamp(1)
            timer.timer_start()
            e = util.util_clearAtCurrentFloor(e)
            fsm_setAllLights(e)
            e.behaviour[e.id] = constants.DOOR_OPEN

    return

def fsm_onDoorTimeout(e):
    bh = e.behaviour[e.id]
    if (bh == constants.DOOR_OPEN):
        e.direction[e.id] = util.util_chooseDirection(e)
        driver.elev_set_door_open_lamp(0)
        driver.elev_set_motor_direction(e.direction[e.id])

        if (e.direction[e.id] == constants.DIRN_STOP):
            e.behaviour[e.id] = constants.IDLE
        else:
            e.behaviour[e.id] = constants.MOVING
    return
