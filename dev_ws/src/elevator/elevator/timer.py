import threading
import time

from constants import *

#~ Door Open Timer
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


#~ Order Confirmed Watchdog
def timer_orderConfirmedTimeout(start_time):
    return (time.time() > start_time + TIME_ORDER_TIMEOUT)

def timer_orderConfirmedStart(elev,id,floor,btn):
    elev.unacknowledgedOrders[time.time()] = [id, floor, btn]
    return

def timer_orderConfirmedStop(elev,start_time):
    elev.unacknowledgedOrders.pop(start_time, None)
    return

#~ Heartbeat Timer
heartbeat_active  = 0
heartbeat_endTime = 0

def timer_heartbeatSendTimeout():
    return (heartbeat_active and (time.time() > heartbeat_endTime))

def timer_heartbeatReceiveTimeout(start_time):
    return (time.time() > start_time + TIME_HEARTBEAT_RECEIVE)

def timer_heartbeatRestart():
    global heartbeat_active
    heartbeat_active = 0
    timer_heartbeatStart()
    return

def timer_heartbeatStart():
    global heartbeat_active
    global heartbeat_endTime
    heartbeat_active  = 1
    heartbeat_endTime = time.time() + TIME_HEARTBEAT_SEND
    return

#~ Mechanical Error Watchdog
exec_active  = 0
exec_endTime = 0

def timer_executionTimeout():
    return (exec_active and (time.time() > exec_endTime))

def timer_executionStart():
    global exec_active
    global exec_endTime
    exec_active  = 1
    exec_endTime = time.time() + TIME_EXEC_TIMEOUT

    return

def timer_executionStop():
    global exec_active
    exec_active = 0

    return
