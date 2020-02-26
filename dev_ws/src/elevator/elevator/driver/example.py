from ctypes import *

#Number of floors. Hardware-dependent, do not modify.
N_FLOORS = 4

# Number of buttons (and corresponding lamps) on a per-floor basis
N_BUTTONS = 3

DIRN_DOWN = -1
DIRN_STOP = 0
DIRN_UP = 1

BTN_HALL_UP = 0
BTN_HALL_DOWN = 1
BTN_CAB = 2

elev = cdll.LoadLibrary("./driver.so")

elev.elev_init(0)

print("Press STOP button to stop elevator and exit program.\n")

elev.elev_set_motor_direction(DIRN_UP)

while (True):
    # Change direction when we reach top/bottom floor
    if (elev.elev_get_floor_sensor_signal() == N_FLOORS - 1):
        elev.elev_set_motor_direction(DIRN_DOWN)
    elif (elev.elev_get_floor_sensor_signal() == 0):
        elev.elev_set_motor_direction(DIRN_UP)
    #Stop elevator and exit program if the stop button is pressed
    if (elev.elev_get_stop_signal()):
        elev.elev_set_motor_direction(DIRN_STOP)
        break
