import threading
import constants

#Undersøk denne litt mer

timer_done = 0

def timer_timedOut():
    if (timer_done):
        timer_done = 0
        return 1
    return

def timer_update():
    timer_done = 1
    return

def timer_start():
    timer = threading.Timer(TIME_DOOR_OPEN,timer_update)
    return
