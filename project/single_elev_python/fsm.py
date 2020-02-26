from ctypes import *

import constants
import status
import util
import timer


driver = cdll.LoadLibrary("../driver/driver.so")
driver.elev_init(0)


def fsm_init(e):
    while(driver.elev_get_floor_sensor_signal() == -1):
        driver.elev_set_motor_direction(constants.DIRN_DOWN)
    driver.elev_set_motor_direction(constants.DIRN_STOP)
    e.behaviour = constants.IDLE
    e.floor = driver.elev_get_floor_sensor_signal()
    e.direction = constants.DIRN_STOP
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
    e.direction =DIRN_DOWN
    e.behaviour = MOVING
    return

def fsm_onNewOrder(e,f,b): #When a new order is distributed and confirmed from the cost function

    bh = e.behaviour

    if (bh == constants.DOOR_OPEN):
        if (e.floor == f):
            timer.timer_start()
        else:
            e.queue[e.id][f][b] = 1

    elif (bh == constants.MOVING):
        e.queue[e.id][f][b] = 1

    elif (bh == constants.IDLE):
        if (e.floor == f):
            driver.elev_set_door_open_lamp(1)
            timer.timer_start()
            e.behaviour = constants.DOOR_OPEN
        else:
            e.queue[e.id][f][b] = 1
            e.direction = util.util_chooseDirection(e)
            driver.elev_set_motor_direction(e.direction)
            e.behaviour = constants.MOVING

    fsm_setAllLights(e)
    return

def fsm_onFloorArrival(e,f):
    e.floor = f
    driver.elev_set_floor_indicator(f)
    if (e.behaviour == constants.MOVING):
        if (util.util_shouldStop(e)):
            driver.elev_set_motor_direction(constants.DIRN_STOP)
            #e.direction = constants.DIRN_STOP
            #publish order done message
                #Legge til i callback att lyset for denne etsjen skrur seg av
            driver.elev_set_door_open_lamp(1)
            timer.timer_start()
            e = util.util_clearAtCurrentFloor(e)
            fsm_setAllLights(e)
            e.behaviour = constants.DOOR_OPEN
    return

def fsm_onDoorTimeout(e):
    bh = e.behaviour
    if (bh == constants.DOOR_OPEN):
        e.direction = util.util_chooseDirection(e)
        driver.elev_set_door_open_lamp(0)
        driver.elev_set_motor_direction(e.direction)

        if (e.direction == constants.DIRN_STOP):
            e.behaviour = constants.IDLE
        else:
            e.behaviour = constants.MOVING
    return
