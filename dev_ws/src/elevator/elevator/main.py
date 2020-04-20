#~ python libs
import copy
import numpy as np
import os
import rclpy
import socket
import sys
import time
from ctypes import *

#~ Add script path for ros compiled files to find them
install_dir = os.path.join(os.path.dirname(__file__))
sys.path.append(os.path.abspath(install_dir))

#~ ROS packages
from rclpy.node   import Node
from ros2_msg.msg import Init
from ros2_msg.msg import NodeMsg
from ros2_msg.msg import Status
from ros2_msg.msg import Order
from ros2_msg.msg import OrderExecuted
from ros2_msg.msg import OrderConfirmed

#~ source files
from constants      import *
from distributor    import *
from fsm            import *
from messages       import *
from timer          import *
from events         import *
from util           import *
from elevClass      import Elevator


#~~~ Include driver ~~~#
driver_path = os.path.join(os.path.dirname(__file__), '../../../../../../src/elevator/elevator/driver/driver.so')
driver = cdll.LoadLibrary(os.path.abspath(driver_path))
driver.elev_init(ELEV_MODE) #Simulator / Physical model

#~ Get IP for this computer
try:
    local_ip = util_getLocalIp()
    local_id = local_ip.getsockname()[0][-3:] #last 3 characters
    local_id = local_id.replace('.','')
except:
    print("NO INTERNET!")
    print("NO ELEVATOR TODAY!")
    print("SHUTTING DOWN!")
    exit()


elev = Elevator(int(local_id))

#~~~~~~~~# ROS #~~~~~~~~#
class Communication(Node):

    def __init__(self):
        super().__init__('elev_node_'+str(local_id))

        #~ Creating subscribers for listening to topics
        self.init_subscriber            = self.create_subscription(Init, 'init', self.init_callback, 0)
        self.node_subscriber            = self.create_subscription(NodeMsg, 'node', self.node_callback, 3)
        self.status_subscriber          = self.create_subscription(Status, 'status', self.status_callback, 0)
        self.order_subscriber           = self.create_subscription(Order, 'orders', self.order_callback, 0)
        self.order_executed_subscriber  = self.create_subscription(OrderExecuted, 'executed_orders', self.orderExecuted_callback, 10)
        self.order_confirmed_subscriber = self.create_subscription(OrderConfirmed, 'confirmed_orders', self.orderConfirmed_callback, 10)
        self.heartbeat_subscriber       = self.create_subscription(Status, 'heartbeat', self.heartbeat_callback,0)

        #~ Creating publishers for sending to topics
        self.init_publisher             = self.create_publisher(Init, 'init', 0)
        self.node_publisher             = self.create_publisher(NodeMsg, 'node', 3)
        self.status_publisher           = self.create_publisher(Status, 'status', 0)
        self.order_publisher            = self.create_publisher(Order, 'orders', 10)
        self.order_executed_publisher   = self.create_publisher(OrderExecuted, 'executed_orders', 10)
        self.order_confirmed_publisher  = self.create_publisher(OrderConfirmed, 'confirmed_orders', 10)
        self.heartbeat_publisher        = self.create_publisher(Status, 'heartbeat', 0)

        return

  #~~~ Callback functions ~~~#
    def heartbeat_callback(self, msg):
        elev.network[msg.id] = ONLINE
        elev.heartbeat[msg.id] = time.time()
        msg_update_Elevator(elev, msg)
        return

    #~ Callback for when an elevator initializes
    def init_callback(self, msg):
        if (msg.id == elev.id):
            return
        self.get_logger().warn('Init from id %d received!' %(msg.id))
        msg_update_Elevator(elev, msg)
        if (msg.initmode == RECONNECT) or (msg.id not in elev.queue):
                elev.queue[msg.id]  = [[0 for btn in range(N_BUTTONS)] for flr in range(N_FLOORS)]
        nodeMsg = msg_create_nodeMessage(elev, msg.id, msg.initmode)
        self.node_publisher.publish(nodeMsg)
        return

    #~ Callback for when an elevator responds to a new initialization
    def node_callback(self, msg):
        if (msg.id == elev.id):
            return
        self.get_logger().warn('Node from id %d received!' %(msg.id))
        msg_update_Elevator(elev, msg)
        if (msg.id not in elev.queue):
            elev.queue[msg.id] = [[0 for btn in range(N_BUTTONS)] for flr in range(N_FLOORS)]

        for floor in range(N_FLOORS):
            for btn in range(N_BUTTONS):
                #Mapping 1D -> 2D array
                elev.queue[msg.id][floor][btn] = msg.queue[floor*N_BUTTONS + btn]
            if (events_reobtainCabOrder(elev, msg, floor)):
                fsm_onNewOrder(elev, elev.id, floor, BTN_CAB)

        return

    #~ Callback for when an elevator updates its state
    def status_callback(self, msg):
        if (msg.id == elev.id):
            return

        msg_update_Elevator(elev, msg)
        if (msg.network == OFFLINE):
            for floor in range(N_FLOORS):
                for btn in range(N_BUTTONS):
                    if (events_republishOrder(elev, msg.id, floor, btn)):
                        newOrderMsg = msg_create_newOrderMessage(elev, floor, btn)
                        self.order_publisher.publish(newOrderMsg)
                        elev.queue[msg.id][floor][btn] = 0

        return

    #~ Callback for when an elevator confirms an order
    def orderConfirmed_callback(self, msg):
        for start_time in sorted(elev.unacknowledgedOrders):
            if (events_orderMatch(elev, msg, start_time)):
                id      = elev.unacknowledgedOrders[start_time][0]
                floor   = elev.unacknowledgedOrders[start_time][1]
                btn     = elev.unacknowledgedOrders[start_time][2]
                timer_orderConfirmedStop(elev, start_time)
                fsm_onNewOrder(elev, id, floor, btn)
        return

    #~ Callback for when an elevator successfully executes an order
    def orderExecuted_callback(self, msg):
        self.get_logger().warn('Order executed at ID: %d, Floor: %d' %(msg.id, msg.floor))
        if (msg.id != elev.id):
            fsm_onFloorArrival(elev, msg.id, msg.floor)
        return

    #~ Callback for when an elevator has a new order
    def order_callback(self, msg):
        self.get_logger().info('New order from: %d, Floor: %d, Button: %s\n' %(msg.id, msg.floor, msg.button))

        if (msg.button == BTN_CAB):
            fsm_onNewOrder(elev,msg.id,msg.floor,msg.button)
            self.get_logger().info('New cab order distributed to: %s' %(msg.id))
            return

        else:
            elev_copy   = copy.deepcopy(elev)
            executor_id = distributor_id(elev_copy, msg) #ID of elevator which will take the order
            self.get_logger().warn('New hall order distributed to: %s' %(executor_id))

            if (elev.id == executor_id):
                fsm_onNewOrder(elev, executor_id, msg.floor,msg.button)
                orderConfirmedMsg = msg_create_orderConfirmedMessage(elev, msg.floor, msg.button)
                self.order_confirmed_publisher.publish(orderConfirmedMsg)
                if (elev.behaviour[elev.id] == DOOR_OPEN and elev.floor[elev.id] == msg.floor):
                    orderExecutedMsg = msg_create_orderExecutedMessage(elev)
                    self.order_executed_publisher.publish(orderExecutedMsg)
                statusMsg = msg_create_statusMessage(elev)
                self.status_publisher.publish(statusMsg)

            else:
                timer_orderConfirmedStart(elev, executor_id, msg.floor, msg.button)
        return



#~~~~~~~~# Main function #~~~~~~~~#
def main(args=None):
    global elev
    try:
        com.destroy_node()
        rclpy.shutdown()
        print('Program was not shut down correctly last time.')
    except:
        pass

    fsm_init(elev)
    rclpy.init(args=args)
    com = Communication()
    timer_doorsStart()
    initMsg = msg_create_initMessage(elev, RESTART)

    for i in range(20):
        com.init_publisher.publish(initMsg)
        rclpy.spin_once(com, executor=None, timeout_sec=5)

    heartbeatMsg    = msg_create_heartbeatMessage(elev)
    com.heartbeat_publisher.publish(heartbeatMsg)
    com.get_logger().warn("Init and heartbeat message sent from self!")
    timer_heartbeatStart()

    while (rclpy.ok()):
        rclpy.spin_once(com, executor=None, timeout_sec=0)

        #~ Check for new button push
        for floor in range(N_FLOORS):
            for btn in range(N_BUTTONS):
                if (events_newButtonPush(elev, floor, btn)):
                    if (elev.network[elev.id] == ONLINE):
                        newOrderMsg = msg_create_newOrderMessage(elev, floor, btn)
                        com.order_publisher.publish(newOrderMsg)
                    elif(btn == BTN_CAB):
                        fsm_onNewOrder(elev, elev.id, floor, btn)

        #~ Check floor sensors
        if (events_onNewFloor(elev)):
            fsm_onFloorArrival(elev, elev.id, driver.elev_get_floor_sensor_signal())
            if (elev.behaviour[elev.id] == DOOR_OPEN):
                orderExecutedMsg  = msg_create_orderExecutedMessage(elev)
                statusMsg         = msg_create_statusMessage(elev)
                com.order_executed_publisher.publish(orderExecutedMsg)
                com.status_publisher.publish(statusMsg)

        #~ Doors open-timer timeout
        if (timer_doorsTimeout()):
            timer_doorsStop()
            fsm_onDoorTimeout(elev)
            statusMsg = msg_create_statusMessage(elev)
            com.status_publisher.publish(statusMsg)

        #~ Mechanical error-timer timeout
        if (timer_executionTimeout()):
            com.get_logger().error('Mechanical error!')
            timer_executionStop()
            elev.network[elev.id] = OFFLINE
            statusMsg  = msg_create_statusMessage(elev)
            com.status_publisher.publish(statusMsg)
            rclpy.spin_once(com, executor=None, timeout_sec=0)
            fsm_onMechanicalError(elev)
            initMsg    = msg_create_initMessage(elev, RECONNECT)
            com.init_publisher.publish(initMsg)
            rclpy.spin_once(com, executor=None, timeout_sec=0)

        #~ Checking timeouts on the unconfirmed orders
        for start_time in sorted(elev.unacknowledgedOrders):
            if (timer_orderConfirmedTimeout(start_time)):
                com.get_logger().error('Order Confirmation Timed Out!')
                floor   = elev.unacknowledgedOrders[start_time][1]
                btn     = elev.unacknowledgedOrders[start_time][2]
                fsm_onNewOrder(elev, elev.id, floor, btn)
                timer_orderConfirmedStop(elev, start_time)

        #~ Send heartbeat
        if (elev.network[elev.id] == ONLINE and timer_heartbeatSendTimeout()):
            heartbeatMsg = msg_create_heartbeatMessage(elev)
            com.heartbeat_publisher.publish(heartbeatMsg)
            timer_heartbeatRestart()

        #~ Heartbeat timeout
        for id in sorted(elev.heartbeat):
            if (timer_heartbeatReceiveTimeout(elev.heartbeat[id]) and elev.network[id] == ONLINE):
                com.get_logger().error('Heartbeat not received from ID: %d!' %(id))
                rclpy.spin_once(com, executor=None, timeout_sec=0)
                elev.network[id] = OFFLINE
                if (id == elev.id):
                    continue
                for floor in range(N_FLOORS):
                    for btn in range(N_BUTTONS):
                        if (events_republishOrder(elev, id, floor, btn)):
                            newOrderMsg = msg_create_newOrderMessage(elev, floor, btn)
                            com.order_publisher.publish(newOrderMsg)
                            elev.queue[id][floor][btn] = 0

        #~ Check stop button
        if (driver.elev_get_stop_signal()):
            driver.elev_set_motor_direction(DIRN_STOP)
            break

    com.get_logger().error('Main loop done, shutting down.')
    com.destroy_node()
    rclpy.shutdown()
    return

if __name__ == '__main__':
    main()
