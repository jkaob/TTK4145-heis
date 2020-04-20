from ctypes import *
import os
import sys

from elevClass  import *
from constants  import *
from timer      import *
from util       import *

install_dir = os.path.join(os.path.dirname(__file__))
sys.path.append(os.path.abspath(install_dir))

#~ Load driver
driver_path = os.path.join(os.path.dirname(__file__), '../../../../../../src/elevator/elevator/driver/driver.so')
driver = cdll.LoadLibrary(os.path.abspath(driver_path))


#~ On initialization. Move down to nearest floor  and initialize.
def fsm_init(elev):
        while (driver.elev_get_floor_sensor_signal() == BETWEEN_FLOORS):
            driver.elev_set_motor_direction(DIRN_DOWN)

        driver.elev_set_motor_direction(DIRN_STOP)
        elev.floor[elev.id]         = driver.elev_get_floor_sensor_signal()
        elev.behaviour[elev.id]     = IDLE
        elev.direction[elev.id]     = DIRN_STOP
        elev.network[elev.id]       = ONLINE

        driver.elev_set_floor_indicator(elev.floor[elev.id])
        util_setAllLights(elev)
        return

#~ When a new order is distributed and confirmed from the cost function
def fsm_onNewOrder(elev, id, floor, btn):
        # if another elevator gets the order
        if (id != elev.id):
            elev.queue[id][floor][btn] = 1
            util_setAllLights(elev)
            return

        behaviour = elev.behaviour[elev.id]
        if (behaviour == DOOR_OPEN):
            if (elev.floor[elev.id] == floor):
                timer_doorsStart()
            else:
                elev.queue[elev.id][floor][btn] = 1
        elif (behaviour == MOVING):
            elev.queue[elev.id][floor][btn]     = 1
        elif (behaviour == IDLE):
            if (elev.floor[elev.id] == floor):
                driver.elev_set_door_open_lamp(1)
                timer_doorsStart()
                elev.behaviour[elev.id]         = DOOR_OPEN
            else:
                elev.queue[elev.id][floor][btn] = 1
                elev.direction[elev.id]         = util_chooseDirection(elev)
                elev.behaviour[elev.id]         = MOVING
                driver.elev_set_motor_direction(elev.direction[elev.id])
                timer_executionStart() # check for mechanical error

        util_setAllLights(elev)
        return

#~ When an elevator arrives at a floor
def fsm_onFloorArrival(elev, id, floor):
        # if other elevator arrived at floor
        if (id != elev.id):
            for btn in range(N_BUTTONS):
                elev.queue[id][floor][btn] = 0
            util_setAllLights(elev)
            return

        timer_executionStop() # no mechanical error

        elev.floor[elev.id] = floor
        driver.elev_set_floor_indicator(floor)
        if (elev.behaviour[elev.id] == MOVING):
            if (util_shouldStop(elev)):
                driver.elev_set_motor_direction(DIRN_STOP)
                driver.elev_set_door_open_lamp(1)
                timer_doorsStart()
                util_clearAtCurrentFloor(elev)
                util_setAllLights(elev)
                elev.behaviour[elev.id] = DOOR_OPEN
            else:
                timer_executionStart()

        return

#~ When the doors close
def fsm_onDoorTimeout(elev):
        behaviour = elev.behaviour[elev.id]
        if (behaviour == DOOR_OPEN):
            elev.direction[elev.id]     = util_chooseDirection(elev)
            driver.elev_set_door_open_lamp(0)
            driver.elev_set_motor_direction(elev.direction[elev.id])
            if (elev.direction[elev.id] == DIRN_STOP):
                elev.behaviour[elev.id] = IDLE
            else:
                elev.behaviour[elev.id] = MOVING
                timer_executionStart() # check for mechanical error
        return

#~ When the elevator is unable to reach a floor due to mechanical error
def fsm_onMechanicalError(elev):
        for floor in range(N_FLOORS):
            for btn in range(N_BUTTONS):
                if (btn == BTN_CAB):
                    continue
                elev.queue[elev.id][floor][btn] = 0

        # when error, revert direction and go back to previous floor
        while (driver.elev_get_floor_sensor_signal() == BETWEEN_FLOORS) or (driver.elev_get_floor_sensor_signal() == elev.floor[elev.id]):
            driver.elev_set_motor_direction(elev.direction[elev.id]*(-1))
            if (driver.elev_get_floor_sensor_signal() == 0) or (driver.elev_get_floor_sensor_signal() == N_FLOORS-1):
                break

        driver.elev_set_motor_direction(DIRN_STOP)
        driver.elev_set_door_open_lamp(1)
        driver.elev_set_floor_indicator(elev.floor[elev.id],0)
        timer_doorsStart()
        elev.floor[elev.id]       = driver.elev_get_floor_sensor_signal()
        elev.behaviour[elev.id]   = DOOR_OPEN
        elev.direction[elev.id]   = DIRN_STOP
        elev.network[elev.id]     = ONLINE
        driver.elev_set_floor_indicator(elev.floor[elev.id],1)
        return
