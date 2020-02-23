from ctypes import cdll

import constants
import status
import util
import timer


self = cdll.LoadLibrary("./driver.so")



#def fsm_init():
#    do shit here

#Denne burde kanskje flyttes til et annet sted? Kanskje rename util til utils, og legge til denne der?
def fsm_setAllLights(e): #Call this function when new order is done, as callback, or distributed
    for f in range(N_FLOORS):
        for b in range(N_BUTTONS):
            #for id in id_list: - Implementer dette
                if (e.queue[id][f][b]):
                    self.elev_set_button_lamp(b, f, 1)
                    break
                self.elev_set_button_lamp(b, f, 0)

    return

def fsm_onInitBetweenFloors(e):
    self.elev_set_motor_direction(DIRN_DOWN)
    e.direction =DIRN_DOWN
    e.behaviour = MOVING
    return

def fsm_onNewOrder(e,f,b): #When a new order is distributed and confirmed from the cost function
    e.queue[e.id][f][b] = 1
    fsm_setAllLights(e)
    return

def fsm_onFloorArrival(e,f):
    e.floor = f
    self.elev_set_floor_indicator(f)
    if (e.behaviour == MOVING):
        if (util_shouldStop(e)):
            self.elev_set_motor_direction(DIRN_STOP)
            #publish order done message
                #Legge til i callback att lyset for denne etsjen skrur seg av
            self.elev_set_door_open_lamp(1)
            timer_start()
            e = util_clearAtCurrentFloor(e)
            fsm_setAllLights(e)
            elevator.behaviour = DOOR_OPEN
    return

def fsm_onDoorTimeout(e):
    bh = elevator.behaviour
    if (bh == DOOR_OPEN):
        e.dirn = util_chooseDirection(e)
        self.elev_set_door_open_lamp(0)
        self.elev_set_motor_direction(e.direction)

        if (e.direction == DIRN_STOP):
            e.behaviour = IDLE
        else:
            e.behaviour = MOVING
    return
