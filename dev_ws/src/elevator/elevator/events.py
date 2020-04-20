import os
import elevClass
import sys
from ctypes    import *

from constants import *
from util      import *

driver_path = os.path.join(os.path.dirname(__file__), '../../../../../../src/elevator/elevator/driver/driver.so')
driver = cdll.LoadLibrary(os.path.abspath(driver_path))

def events_onNewFloor(elev):
    floor_sensor = driver.elev_get_floor_sensor_signal()
    return ((floor_sensor != BETWEEN_FLOORS) and (floor_sensor != elev.floor[elev.id]))

def events_newButtonPush(elev, floor, btn):
    v = driver.elev_get_button_signal(btn, floor)
    if (v and (v != elev.queue[elev.id][floor][btn])):
        while (driver.elev_get_button_signal(btn, floor)):
            pass
        return True
    return False

#~ Returns True if this elevator lost an old cab order
def events_reobtainCabOrder(elev, msg, floor):
    return (msg.initmode == RESTART) and (msg.cabqueue[floor] == 1) and (msg.knownid == elev.id)

def events_republishOrder (elev, id, floor, btn):
    return (btn != BTN_CAB and elev.queue[id][floor][btn] == 1)

#~ Returns True if "Order confirmed" message matches an order in the order history
def events_orderMatch(elev, msg, start_time):
    id      = elev.unacknowledgedOrders[start_time][0]
    floor   = elev.unacknowledgedOrders[start_time][1]
    btn     = elev.unacknowledgedOrders[start_time][2]

    return (msg.id == id and msg.floor == floor and msg.button == btn)
