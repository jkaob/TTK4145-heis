import status
import os
import sys
import platform    # For getting the operating system name
import subprocess  # For executing a shell command
from ctypes import *
from constants import *

###### LOAD DRIVER ########
driver_path = os.path.join(os.path.dirname(__file__), '../../../../../../src/elevator/elevator/driver/driver.so')
driver = cdll.LoadLibrary(os.path.abspath(driver_path))
#driver.elev_init(ELEV_MODE) #Simulator / Physical model
###########################

def util_orderAbove(elev): #Ta inn status klassen til seg selv
    for f in range(elev.floor[elev.id] + 1, N_FLOORS):
        for b in range(N_BUTTONS):
            if (elev.queue[elev.id][f][b]):
                return 1
    return 0

def util_orderBelow(elev): #Ta inn status klassen til seg selv
    for f in range(0, elev.floor[elev.id]):
        for b in range(N_BUTTONS):
            if (elev.queue[elev.id][f][b]):
                return 1
    return 0

def util_chooseDirection(elev):
    dir = elev.direction[elev.id]
    if (dir == DIRN_UP):
        if (util_orderAbove(elev)):
            return DIRN_UP
        elif (util_orderBelow(elev)):
            return DIRN_DOWN
        else:
            return DIRN_STOP

    elif ((dir == DIRN_DOWN) or (dir == DIRN_STOP)):
        if (util_orderBelow(elev)):
            return DIRN_DOWN
        elif (util_orderAbove(elev)):
            return DIRN_UP
        else:
            return DIRN_STOP

    else:
        return DIRN_STOP

def util_shouldStop(elev):
    dir = elev.direction[elev.id]
    if (dir == DIRN_DOWN):
        return ((elev.queue[elev.id][elev.floor[elev.id]][BTN_HALL_DOWN]) or
                (elev.queue[elev.id][elev.floor[elev.id]][BTN_CAB]) or
                not(util_orderBelow(elev)))
    elif (dir == DIRN_UP):
        return ((elev.queue[elev.id][elev.floor[elev.id]][BTN_HALL_UP]) or
                (elev.queue[elev.id][elev.floor[elev.id]][BTN_CAB]) or
                not(util_orderAbove(elev)))
    else:
        return 1

def util_clearAtCurrentFloor(elev):
    for b in range(N_BUTTONS):
        if (elev.queue[elev.id][elev.floor[elev.id]][b]):
            elev.queue[elev.id][elev.floor[elev.id]][b] = 0
    return


def util_setAllLights(elev): #Call this function when new order is done, as callback, or distributed
    for f in range(N_FLOORS):
        for b in range(N_BUTTONS):
            for id in sorted(elev.queue):
                if (b == BTN_CAB and id != elev.id):
                    continue
                elif (elev.queue[id][f][b]):
                    driver.elev_set_button_lamp(b, f, 1)
                    break
                else:
                    driver.elev_set_button_lamp(b, f, 0)
    return

def util_ping(target_ip):
    """
    Returns True if host (str) responds to a ping request.
    Remember that a host may not respond to a ping (ICMP) request even if the host name is valid.
    """

    # Option for the number of packets as a function of
    param = '-c'

    # Building the command. Ex: "ping -c 1 google.com"
    command = ['ping', param, '1', target_ip]

    return subprocess.call(command, stdout=subprocess.PIPE)
