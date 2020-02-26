import timer
import fsm
import constants
from status import LocalElevator

elev = LocalElevator()


def main():
    global elev

    fsm.fsm_init(elev)

    while(1):

        ## Request button
        for f in range(constants.N_FLOORS):
            for b in range(constants.N_BUTTONS):
                v = fsm.driver.elev_get_button_signal(b,f)
                if(v and (v != elev.queue[elev.id][f][b])):
                    fsm.fsm_onNewOrder(elev,f,b)

        ## Floor sensor
        f = fsm.driver.elev_get_floor_sensor_signal()
        if(f != -1 and f != elev.floor):
            fsm.fsm_onFloorArrival(elev,f)

        ## Timer
        if(timer.timer_timedOut()):
            timer.timer_stop()
            fsm.fsm_onDoorTimeout(elev)


        ##Stop button
        if (fsm.driver.elev_get_stop_signal()):
            fsm.driver.elev_set_motor_direction(constants.DIRN_STOP)
            break
            
main()
