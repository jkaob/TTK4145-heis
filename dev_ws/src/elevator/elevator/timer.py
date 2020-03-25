import threading
import time

from constants import *


#Undersøk denne litt mer

#DOOR TIMER
active = 0
end_time = 0

def timer_timedOut():
    return (active and (time.time()>end_time))

def timer_stop():
    global active
    active = 0
    return


def timer_start():
    global end_time
    global active
    end_time = time.time()+TIME_DOOR_OPEN
    active = 1
    return


# ORDER WATCHDOG TIMER
def timer_orderConfirmedTimeout(start_time):
    return (time.time()>start_time+TIME_ORDER_TIMEOUT)

def timer_orderConfirmedStop(e,start_time):
    e.unacknowledgedOrders.pop(start_time, None)
    return


def timer_orderConfirmedStart(e,id,f,btn):
    e.unacknowledgedOrders[time.time()] = [id, f, btn]
    return
