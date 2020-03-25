import rclpy

from ros2_msg.msg import Order
from ros2_msg.msg import OrderConfirmed
from ros2_msg.msg import Status

from status import LocalElevator
from constants import *

def messages_createMessage(e, type, floor=None, btn=None, knownID=None, initmode=None):
    if (type == MSG_INIT):
        msg           = Status()
        msg.id        = e.id
        msg.floor     = e.floor[e.id]
        msg.behaviour = e.behaviour[e.id]
        msg.direction = e.direction[e.id]
        msg.network   = e.network[e.id]
        msg.initmode  = initmode

    elif (type == MSG_NEW_ORDER or type == MSG_ORDER_CONFIRMED):
        msg           = Order()
        msg.id        = e.id
        msg.floor     = floor
        msg.button    = btn

    elif (type == MSG_ORDER_EXECUTED):
        msg           = Order()
        msg.id        = e.id
        msg.floor     = e.floor[e.id]

    elif (type == MSG_STATUS):
        msg           = Status()
        msg.id        = e.id
        msg.floor     = e.floor[e.id]
        msg.behaviour = e.behaviour[e.id]
        msg.direction = e.direction[e.id]
        msg.network   = e.network[e.id]

    elif (type == MSG_NODE):
        msg           = Status()
        msg.id        = e.id
        msg.behaviour = e.behaviour[e.id]
        msg.direction = e.direction[e.id]
        msg.floor     = e.floor[e.id]
        msg.network   = e.network[e.id]
        msg.initmode  = initmode
        msg.knownid   = knownID
        msg.queue     = [0 for i in range(N_BUTTONS * N_FLOORS)]
        msg.cabqueue  = [0 for i in range(N_FLOORS)]

        for f in range(N_FLOORS):
            for b in range(N_BUTTONS):
                index = f*N_BUTTONS + b
                msg.queue[index] = int(e.queue[e.id][f][b])
            msg.cabqueue[f] = e.queue[knownID][f][BTN_CAB]
    return msg
