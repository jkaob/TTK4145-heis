import rclpy

from ros2_msg.msg import Order
from ros2_msg.msg import OrderConfirmed
from ros2_msg.msg import Status

from status import LocalElevator
import constants

def messages_createMessage(e, type, f=None, b=None):
    if (type == constants.MSG_INIT):
        msg = Status()
        msg.id = e.id
        msg.behaviour = e.behaviour[e.id]
        msg.direction = e.direction[e.id]
        msg.floor = e.floor[e.id]
    elif (type == constants.MSG_NEW_ORDER or type == constants.MSG_ORDER_CONFIRMED):
        msg = Order()
        msg.id = e.id
        msg.floor = f
        msg.button = b
    elif (type == constants.MSG_ORDER_EXECUTED):
        msg          = Order()
        msg.id       = e.id
        msg.floor    = e.floor[e.id]
    elif (type == constants.MSG_STATUS):
        msg = Status()
        msg.id = e.id
        msg.behaviour = e.behaviour[e.id]
        msg.direction = e.direction[e.id]
        msg.floor = e.floor[e.id]
    elif (type == constants.MSG_NODE):
        msg = Status()
        msg.id = e.id
        msg.behaviour = e.behaviour[e.id]
        msg.direction = e.direction[e.id]
        msg.floor = e.floor[e.id]
        msg.queue = [0 for i in range(constants.N_BUTTONS * constants.N_FLOORS)];
        for f in range(constants.N_FLOORS):
            for b in range(constants.N_BUTTONS):
                index = f*constants.N_BUTTONS + b
                msg.queue[index] = int(e.queue[e.id][f][b])
    return msg
