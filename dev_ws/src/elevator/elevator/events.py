import os
import status
import sys
from ctypes    import *

from constants import *
from util      import *

driver_path = os.path.join(os.path.dirname(__file__), '../../../../../../src/elevator/elevator/driver/driver.so')
driver = cdll.LoadLibrary(os.path.abspath(driver_path))
#driver.elev_init(ELEV_MODE) #Simulator / Physical model


def events_onNewFloor(elev):
    floor_sensor = driver.elev_get_floor_sensor_signal()
    return ((floor_sensor != BETWEEN_FLOORS) and (floor_sensor != elev.floor[elev.id]))

def events_newButtonPush(elev, floor, btn):
    v = driver.elev_get_button_signal(btn, floor)
    if (v and (v != elev.queue[elev.id][floor][btn])):
        while (driver.elev_get_button_signal(btn, floor)):
            pass ## sjekk om dette kan gjøres bedre
        return True
    return False

def events_offlineAndNoOrders(elev):
    return ((elev.network[elev.id] == OFFLINE) and (elev.behaviour[elev.id] == IDLE))

def events_otherElevOffline(elev, id):
    target_ip = util_getTargetIp(elev,id)
    response = util_ping(target_ip)
    return ((response == OFFLINE) and (elev.network[id] == ONLINE))

def events_getOldCabOrders(elev, msg, floor):
    return (msg.initmode == RESTART) and (msg.cabqueue[floor] == 1) and (msg.knownid == elev.id)

def events_republishOrder (elev, id, floor, btn):
    return (btn != BTN_CAB and elev.queue[id][floor][btn] == 1)

def events_orderMatch(elev, msg, start_time):
    id      = elev.unacknowledgedOrders[start_time][0]
    floor   = elev.unacknowledgedOrders[start_time][1]
    btn     = elev.unacknowledgedOrders[start_time][2]

    return (msg.id == id and msg.floor == floor and msg.button == btn)
