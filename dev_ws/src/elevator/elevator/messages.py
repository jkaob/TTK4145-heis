import rclpy

#~ Message formats
from ros2_msg.msg import Init
from ros2_msg.msg import NodeMsg
from ros2_msg.msg import Status
from ros2_msg.msg import Order
from ros2_msg.msg import OrderExecuted
from ros2_msg.msg import OrderConfirmed

from status import LocalElevator
from constants import *

def msg_create_initMessage(elev, initmode):
        msg           = Init()
        msg.id        = elev.id
        msg.floor     = elev.floor[elev.id]
        msg.behaviour = elev.behaviour[elev.id]
        msg.direction = elev.direction[elev.id]
        msg.network   = elev.network[elev.id]
        msg.initmode  = initmode
        return  msg

def msg_create_newOrderMessage(elev, floor, btn):
        msg           = Order()
        msg.id        = elev.id
        msg.floor     = floor
        msg.button    = btn
        return msg

def msg_create_orderConfirmedMessage(elev, floor, btn):
        msg           = OrderConfirmed()
        msg.id        = elev.id
        msg.floor     = floor
        msg.button    = btn
        return msg

def msg_create_orderExecutedMessage(elev):
        msg           = OrderExecuted()
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

def msg_create_nodeMessage(elev, knownID, initmode):
        msg           = NodeMsg()
        msg.id        = elev.id
        msg.floor     = elev.floor[elev.id]
        msg.behaviour = elev.behaviour[elev.id]
        msg.direction = elev.direction[elev.id]
        msg.network   = elev.network[elev.id]
        msg.initmode  = initmode
        msg.knownid   = knownID
        msg.queue     = [0 for i in range(N_BUTTONS * N_FLOORS)]
        msg.cabqueue  = [0 for i in range(N_FLOORS)]

        for f in range(N_FLOORS):
            for b in range(N_BUTTONS):
                msg.queue[f*N_BUTTONS + b] = int(elev.queue[elev.id][f][b])
            msg.cabqueue[f] = elev.queue[knownID][f][BTN_CAB]

        return msg
