from ctypes import *
import os
import status
import sys

driver_path = os.path.join(os.path.dirname(__file__), '../../../../../../src/elevator/elevator/driver/driver.so')
driver = cdll.LoadLibrary(os.path.abspath(driver_path))
#driver.elev_init(ELEV_MODE) #Simulator / Physical model


def events_onNewFloor(elev):
    floor_sensor = driver.elev_get_floor_sensor_signal()
    return ((floor_sensor != -1) and (floor_sensor != elev.floor[elev.id]))

def events_NewButtonPush(elev,floor,btn):
    v = driver.elev_get_button_signal(btn, floor)
    if (v and (v != elev.queue[elev.id][floor][btn])):
        while (driver.elev_get_button_signal(btn, floor)):
            pass ## sjekk om dette kan gjøres bedre
        return True
    return False
