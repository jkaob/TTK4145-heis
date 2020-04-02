import rclpy

from ros2_msg.msg import Order
from ros2_msg.msg import OrderConfirmed
from ros2_msg.msg import Status

from status import LocalElevator
from constants import *

## REMOVE THIS FUNCTION:
def messages_createMessage(e, type, floor=None, btn=None, knownID=None, initmode=None):
    if (type == MSG_INIT):
        msg           = Status()
        msg.id        = elev.id
        msg.floor     = elev.floor[elev.id]
        msg.behaviour = elev.behaviour[elev.id]
        msg.direction = elev.direction[elev.id]
        msg.network   = elev.network[elev.id]
        msg.initmode  = initmode

    elif (type == MSG_NEW_ORDER or type == MSG_ORDER_CONFIRMED):
        msg           = Order()
        msg.id        = elev.id
        msg.floor     = floor
        msg.button    = btn

    elif (type == MSG_ORDER_EXECUTED):
        msg           = Order()
        msg.id        = elev.id
        msg.floor     = elev.floor[elev.id]

    elif (type == MSG_STATUS):
        msg           = Status()
        msg.id        = elev.id
        msg.floor     = elev.floor[elev.id]
        msg.behaviour = elev.behaviour[elev.id]
        msg.direction = elev.direction[elev.id]
        msg.network   = elev.network[elev.id]

    elif (type == MSG_NODE):
        msg           = Status()
        msg.id        = elev.id
        msg.behaviour = elev.behaviour[elev.id]
        msg.direction = elev.direction[elev.id]
        msg.floor     = elev.floor[elev.id]
        msg.network   = elev.network[elev.id]
        msg.initmode  = initmode
        msg.knownid   = knownID
        msg.queue     = [0 for i in range(N_BUTTONS * N_FLOORS)]
        msg.cabqueue  = [0 for i in range(N_FLOORS)]

        for f in range(N_FLOORS):
            for b in range(N_BUTTONS):
                index = f*N_BUTTONS + b
                msg.queue[index] = int(elev.queue[elev.id][f][b])
            msg.cabqueue[f] = elev.queue[knownID][f][BTN_CAB]
    return msg


## THESE ARE THE RIGHT ONES:

def msg_create_initMessage(elev, initmode=None):
        msg           = Status()
        msg.id        = elev.id
        msg.floor     = elev.floor[elev.id]
        msg.behaviour = elev.behaviour[elev.id]
        msg.direction = elev.direction[elev.id]
        msg.network   = elev.network[elev.id]
        msg.initmode  = initmode
        return  msg

def msg_create_newOrderMessage(elev, floor=None, btn=None):
        msg           = Order()
        msg.id        = elev.id
        msg.floor     = floor
        msg.button    = btn
        return msg

def msg_create_orderConfirmedMessage(elev, floor=None, btn=None):
        msg           = Order()
        msg.id        = elev.id
        msg.floor     = floor
        msg.button    = btn
        return msg

def msg_create_orderExecutedMessage(elev):
        msg           = Order()
        msg.id        = elev.id
        msg.floor     = elev.floor[elev.id]
        return msg

def msg_create_statusMessage(elev):
        msg           = Status()
        msg.id        = elev.id
        msg.floor     = elev.floor[elev.id]
        msg.behaviour = elev.behaviour[elev.id]
        msg.direction = elev.direction[elev.id]
        msg.network   = elev.network[elev.id]
        return msg

def msg_create_nodeMessage(elev, knownID=None, initmode=None):
        msg           = Status()
        msg.id        = elev.id
        msg.behaviour = elev.behaviour[elev.id]
        msg.direction = elev.direction[elev.id]
        msg.floor     = elev.floor[elev.id]
        msg.network   = elev.network[elev.id]
        msg.initmode  = initmode
        msg.knownid   = knownID
        msg.queue     = [0 for i in range(N_BUTTONS * N_FLOORS)]
        msg.cabqueue  = [0 for i in range(N_FLOORS)]

        for f in range(N_FLOORS):
            for b in range(N_BUTTONS):
                index = f*N_BUTTONS + b
                msg.queue[index] = int(elev.queue[elev.id][f][b])
            msg.cabqueue[f] = elev.queue[knownID][f][BTN_CAB]

        return msg
