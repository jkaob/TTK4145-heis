import rclpy

from ros2_msg.msg import Order
from ros2_msg.msg import OrderConfirmed
from ros2_msg.msg import Status

from status import LocalElevator
import constants


def messages_createNodeMsg(e):
    node_msg = Status()
    node_msg.id = e.id
    node_msg.behaviour = e.behaviour[e.id]
    node_msg.direction = e.direction[e.id]
    node_msg.floor = e.floor[e.id]
    node_msg.queue = [0 for i in range(constants.N_BUTTONS * constants.N_FLOORS)];
    for f in range(constants.N_FLOORS):
        for b in range(constants.N_BUTTONS):
            index = f*constants.N_BUTTONS + b
            node_msg.queue[index] = int(e.queue[e.id][f][b])
    return node_msg

def messages_createOrderConfirmedMsg(e,f,b):
    order_confirmed_msg = Order()
    order_confirmed_msg.id = e.id
    order_confirmed_msg.floor = f
    order_confirmed_msg.button = b
    return order_confirmed_msg

def messages_createOrderExecutedMsg(e):
    order_OrderExecutedMsg          = Order()
    order_OrderExecutedMsg.id       = e.id
    order_OrderExecutedMsg.floor    = e.floor[e.id]
    return order_OrderExecutedMsg

def messages_createStatusMsg(e):
    status_msg = Status()
    status_msg.id = e.id
    status_msg.behaviour = e.behaviour[e.id]
    status_msg.direction = e.direction[e.id]
    status_msg.floor = e.floor[e.id]
    return status_msg
