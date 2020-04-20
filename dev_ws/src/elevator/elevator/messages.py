import rclpy

#~ Message formats
from ros2_msg.msg import Init
from ros2_msg.msg import NodeMsg
from ros2_msg.msg import Status
from ros2_msg.msg import Order
from ros2_msg.msg import OrderExecuted
from ros2_msg.msg import OrderConfirmed

#~ Source files
from elevClass import Elevator
from constants import *


#~ Creates message to send on initialization
def msg_create_initMessage(elev, initmode):
        msg             = Init()
        msg.id          = elev.id
        msg.floor       = elev.floor[elev.id]
        msg.behaviour   = elev.behaviour[elev.id]
        msg.direction   = elev.direction[elev.id]
        msg.network     = elev.network[elev.id]
        msg.initmode    = initmode

        return  msg

#~ Creates message to send on New order-event
def msg_create_newOrderMessage(elev, floor, btn):
        msg             = Order()
        msg.id          = elev.id
        msg.floor       = floor
        msg.button      = btn

        return msg

#~ Creates message to send when order is confirmed
def msg_create_orderConfirmedMessage(elev, floor, btn):
        msg             = OrderConfirmed()
        msg.id          = elev.id
        msg.floor       = floor
        msg.button      = btn

        return msg

#~ Creates message to send when order at a floor is executed
def msg_create_orderExecutedMessage(elev):
        msg             = OrderExecuted()
        msg.id          = elev.id
        msg.floor       = elev.floor[elev.id]

        return msg

#~ Creates message with status update
def msg_create_statusMessage(elev):
        msg             = Status()
        msg.id          = elev.id
        msg.floor       = elev.floor[elev.id]
        msg.behaviour   = elev.behaviour[elev.id]
        msg.direction   = elev.direction[elev.id]
        msg.network     = elev.network[elev.id]

        return msg

#~ Creates message in response to initialization of another elevator.
#  Replies with its entire queue, as well as the other's cab-queue (if any)
def msg_create_nodeMessage(elev, knownID, initmode):
        msg             = NodeMsg()
        msg.id          = elev.id
        msg.floor       = elev.floor[elev.id]
        msg.behaviour   = elev.behaviour[elev.id]
        msg.direction   = elev.direction[elev.id]
        msg.network     = elev.network[elev.id]
        msg.initmode    = initmode
        msg.knownid     = knownID
        msg.queue       = [0 for i in range(N_BUTTONS * N_FLOORS)]
        msg.cabqueue    = [0 for i in range(N_FLOORS)]

        for floor in range(N_FLOORS):
            for btn in range(N_BUTTONS):
                msg.queue[floor*N_BUTTONS + btn] = int(elev.queue[elev.id][floor][btn]) #Mapping 2D -> 1D array
            msg.cabqueue[floor] = elev.queue[knownID][floor][BTN_CAB]

        return msg

#~ Creates heartbeat message to send regurarly
def msg_create_heartbeatMessage(elev):
        return msg_create_statusMessage(elev)

#~ Copies data from message into the elevator
def msg_update_Elevator(elev, msg):
        elev.floor[msg.id]      = msg.floor
        elev.behaviour[msg.id]  = msg.behaviour
        elev.direction[msg.id]  = msg.direction
        elev.network[msg.id]    = msg.network

        return
