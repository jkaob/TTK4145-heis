from ctypes import cdll

import constants
import status
import util
import timer

elev = LocalElevator()

self = cdll.LoadLibrary("./driver.so")



#def fsm_init():
#    do shit here

#Denne burde kanskje flyttes til et annet sted? Kanskje rename util til utils, og legge til denne der?
def fsm_setAllLights(): #Call this function when new order is done, as callback, or distributed
    for f in range(N_FLOORS):
        for b in range(N_BUTTONS):
            #for id in id_list: - Implementer dette
                if (elev.queue[id][f][b]):
                    self.elev_set_button_lamp(b, f, 1)
                    break
                self.elev_set_button_lamp(b, f, 0)

    return

def fsm_onInitBetweenFloors():
    self.elev_set_motor_direction(DIRN_DOWN)
    elev.direction =DIRN_DOWN
    elev.behaviour = MOVING
    return

def fsm_onNewOrder(id,f,b): #When a new order is distributed and confirmed from the cost function
    elev.queue[id][f][b] = 1
    fsm_setAllLights()
    return

def fsm_onFloorArrival(f):
    elev.floor = f
    self.elev_set_floor_indicator(f)
    if (elev.behaviour == MOVING):
        if (util_shouldStop(elev)):
            self.elev_set_motor_direction(DIRN_STOP)
            #publish order done message
                #Legge til i callback att lyset for denne etsjen skrur seg av
            self.elev_set_door_open_lamp(1)
            timer_start()
            util_clearAtCurrentFloor(elev)
            fsm_setAllLights()
            elevator.behaviour = DOOR_OPEN
    return

def fsm_onDoorTimeout():
    bh = elevator.behaviour
    if (bh == DOOR_OPEN):
        elev.dirn = util_chooseDirection(elev)
        self.elev_set_door_open_lamp(0)
        self.elev_set_motor_direction(elev.direction)

        if (elev.direction == DIRN_STOP):
            elev.behaviour = IDLE
        else:
            elev.behaviour = MOVING
    return
