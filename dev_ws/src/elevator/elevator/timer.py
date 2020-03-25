import threading
import time

from constants import *
#Undersøk denne litt mer



#~ Door Open Watchdog Timer
doors_active  = 0
doors_endTime = 0

def timer_doorsTimeout():
    return (doors_active and (time.time() > doors_endTime))

def timer_doorsStop():
    global doors_active
    doors_active = 0
    return

def timer_doorsStart():
    global doors_active
    global doors_endTime
    doors_active  = 1
    doors_endTime = time.time() + TIME_DOOR_OPEN

    return


#~ Order Confirmed Watchdog Timer
def timer_orderConfirmedTimeout(start_time):
    return (time.time() > start_time + TIME_ORDER_TIMEOUT)

def timer_orderConfirmedStart(e,id,f,btn):
    e.unacknowledgedOrders[time.time()] = [id, f, btn]
    return

def timer_orderConfirmedStop(e,start_time):
    e.unacknowledgedOrders.pop(start_time, None)
    return


#~ Mechanical Error Watchdog Timer
exec_active  = 0
exec_endTime = 0

def timer_executionTimeout():
    return (exec_active and (time.time() > exec_endTime))

def timer_executionStart():
    global exec_active
    global exec_endTime
    exec_active  = 1
    exec_endTime = time.time() + TIME_EXEC_TIMEOUT
    print('Execution timer started')

    return

def timer_executionStop():
    global exec_active
    exec_active = 0
    print('Execution timer stopped')

    return
