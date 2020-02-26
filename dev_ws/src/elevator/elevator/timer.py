import threading
import constants
import time

#Undersøk denne litt mer

timer_active = 0
end_time = 0

def timer_timedOut():
    return (timer_active and (time.time()>end_time))

def timer_stop():
    global timer_active
    timer_active = 0
    return


def timer_start():
    global end_time
    global timer_active
    end_time = time.time()+constants.TIME_DOOR_OPEN
    timer_active = 1
    return
