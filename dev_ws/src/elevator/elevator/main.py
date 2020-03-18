import sys
import os
#### Add script path for ros compiled files to find them ###
install_dir = os.path.join(os.path.dirname(__file__))
sys.path.append(os.path.abspath(install_dir))
############################################################

import timer
import time
import fsm
import constants
import socket
import rclpy
import distributor
import copy
from status import LocalElevator
from statusCopy import SingleElevatorCopy

from rclpy.node import Node
from std_msgs.msg import String
from ros2_msg.msg import Order
from ros2_msg.msg import OrderConfirmed
from ros2_msg.msg import Status


import numpy as np


## Get ip this computer ##
s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
s.connect(("8.8.8.8", 80))
local_id = s.getsockname()[0][-3:]
###########################


elev = LocalElevator(int(local_id))


#######ROS########
# For å sende en ordre må callback kjøre fsm_onNewOrder(msg.id,msg.floor,msg.button)
class OrderNode(Node):

    def __init__(self):
        super().__init__('order_node_'+str(local_id))
        #Subscribersconfirmed
        self.order_subscriber = self.create_subscription(Order, 'orders', self.order_callback, 10)
        self.order_confirmed_subscriber = self.create_subscription(Order, 'confirmed_orders', self.order_confirmed_callback, 10)
        self.order_executed_subscriber = self.create_subscription(Order, 'executed_orders', self.order_executed_callback, 10)
        self.status_subscriber = self.create_subscription(Status, 'status', self.status_callback, 10)
        self.init_subscriber = self.create_subscription(Status, 'init', self.init_callback, 10)
        self.node_subscriber = self.create_subscription(Status, 'node', self.node_callback, 10)

        #Publishers
        self.order_publisher = self.create_publisher(Order, "orders", 10)
        self.order_confirmed_publisher = self.create_publisher(Order,'confirmed_orders', 10)
        self.order_executed_publisher = self.create_publisher(Order,'executed_orders', 10)
        self.status_publisher = self.create_publisher(Status, 'status', 10)
        self.init_publisher = self.create_publisher(Status, 'init', 10)
        self.node_publisher = self.create_publisher(Status, 'node', 10)
    #Callback functions

    def node_callback(self,msg):
        elev.floor[msg.id] = msg.floor
        elev.behaviour[msg.id] = msg.behaviour
        elev.direction[msg.id] = msg.direction
        if (msg.id not in elev.queue):
            order_node.get_logger().warn('Elevator id not in que! Id: %d\n' %(msg.id))
            matrix = [[0 for b in range(constants.N_BUTTONS)] for f in range(constants.N_FLOORS)]
            elev.queue[msg.id] = matrix
        for f in range(constants.N_FLOORS):
            for b in range(constants.N_BUTTONS):
                index = f*constants.N_BUTTONS + b
                elev.queue[msg.id][f][b] = msg.queue[index]
        for id in elev.queue:
            print(id)
            print(elev.queue[id])

    def init_callback(self,msg):
        elev.floor[msg.id] = msg.floor
        elev.behaviour[msg.id] = msg.behaviour
        elev.direction[msg.id] = msg.direction
        matrix = [[0 for b in range(constants.N_BUTTONS)] for f in range(constants.N_FLOORS)]
        elev.queue[msg.id] = matrix

        node_msg = Status()
        node_msg.id = elev.id
        node_msg.behaviour = elev.behaviour[elev.id]
        node_msg.direction = elev.direction[elev.id]
        node_msg.floor = elev.floor[elev.id]
        node_msg.queue = [0 for i in range(constants.N_BUTTONS * constants.N_FLOORS)];

        for f in range(constants.N_FLOORS):
            for b in range(constants.N_BUTTONS):
                index = f*constants.N_BUTTONS + b
                node_msg.queue[index] = int(elev.queue[elev.id][f][b])

        self.node_publisher.publish(node_msg)


    def status_callback(self, msg):

        elev.floor[msg.id] = msg.floor
        elev.behaviour[msg.id] = msg.behaviour
        elev.direction[msg.id] = msg.direction

    def order_confirmed_callback(self, msg):
        for start_time in sorted(elev.unacknowledgedOrders):
            id = elev.unacknowledgedOrders[start_time][0]
            f = elev.unacknowledgedOrders[start_time][1]
            b = elev.unacknowledgedOrders[start_time][2]
            if(msg.id == id and msg.floor == f and msg.button == b):
                timer.timer_orderConfirmedStop(elev,start_time)
                fsm.fsm_onNewOrder(elev,id,f,b)
        #Print all queues
        for id in elev.queue:
            print("Id: %d\t" %(id), end = " ")
            print(elev.queue[id])
        print(" --- \n")

    def order_executed_callback(self, msg):
        if (msg.id != elev.id):
            fsm.fsm_onFloorArrival(elev, msg.id, msg.floor)

    def order_callback(self, msg):
        self.get_logger().info('New order from: %d\n Floor: %d\n Button: %s' %(msg.id, msg.floor, msg.button))

        if (msg.button == constants.BTN_CAB):
            fsm.fsm_onNewOrder(elev,msg.id,msg.floor,msg.button)
            self.get_logger().info('New cab order distributed to: %s\n' %(msg.id))
            return
        else:
#################### ORDER DISTRIBUTER ASSIGNS NEW ORDER #########################
            min_duration = 999
            min_id = 0

            #print("Før kopi")
            # print(elev.queue[elev.id])
            # print('\n')

            elev_copy = copy.deepcopy(elev)

            #print("Etter kopi")
            # print(elev.queue[elev.id])
            # print('\n')

            for id in sorted(elev_copy.queue):

                f = elev_copy.floor[id]
                bh = elev_copy.behaviour[id]
                dir = elev_copy.direction[id]
                qu = elev_copy.queue[id]

                single_elev_copy = SingleElevatorCopy(id, f, bh, dir, qu)
                single_elev_copy.queue[id][msg.floor][msg.button] = 1

                # print("Single elev copy")
                # print(single_elev_copy.queue[id])
                # print('\n')

                duration = distributor.distributor_timeToIdle(single_elev_copy)
                self.get_logger().warn('ID: %s      Duration: %d\n' %(id, duration))
                if(duration < min_duration):
                    min_duration = duration
                    min_id = id
    ###################################################################################
            self.get_logger().info('New hall order distributed to: %s\n' %(min_id))

            if (elev.id == min_id):
                fsm.fsm_onNewOrder(elev, min_id, msg.floor,msg.button)
                order_confirmed_msg = Order()
                order_confirmed_msg.id = elev.id
                order_confirmed_msg.floor = msg.floor
                order_confirmed_msg.button = msg.button
                ##publish order received to reset timer
                self.order_confirmed_publisher.publish(order_confirmed_msg)

                status_msg = Status()
                status_msg.id = elev.id
                status_msg.behaviour = elev.behaviour[elev.id]
                status_msg.direction = elev.direction[elev.id]
                status_msg.floor = elev.floor[elev.id]

                self.status_publisher.publish(status_msg)

            else:
                #Add to unacknowledgedOrders
                timer.timer_orderConfirmedStart(elev, min_id, msg.floor, msg.button)


############### MAIN LOOP #####################
def main(args=None):
    rclpy.init(args=args)
    order_node = OrderNode()

    global elev
    fsm.fsm_init(elev)
    order_node.get_logger().warn('Init complete! Id: %d\n' %(elev.id))

    init_msg = Status()
    init_msg.id = elev.id
    init_msg.behaviour = elev.behaviour[elev.id]
    init_msg.direction = elev.direction[elev.id]
    init_msg.floor = elev.floor[elev.id]
    order_node.init_publisher.publish(init_msg)

    rclpy.spin_once(order_node,executor=None,timeout_sec=0)

    order_node.get_logger().warn("Init message sent!")

    while(rclpy.ok()):
        #order_node.get_logger().info('Her')
        rclpy.spin_once(order_node,executor=None,timeout_sec=0)
        #order_node.get_logger().info('Forbi')

        ## Request button
        for f in range(constants.N_FLOORS):
            for b in range(constants.N_BUTTONS):
                v = fsm.driver.elev_get_button_signal(b,f)
                if(v and (v != elev.queue[elev.id][f][b])):
                    while ( fsm.driver.elev_get_button_signal(b,f) ):
                        pass
                    order_newOrderMsg           = Order()
                    order_newOrderMsg.id        = elev.id
                    order_newOrderMsg.floor     = f
                    order_newOrderMsg.button    = b
                    order_node.order_publisher.publish(order_newOrderMsg)
                    #fsm.fsm_onNewOrder(elev,elev.id,f,b)

        ## Floor sensor
        f = fsm.driver.elev_get_floor_sensor_signal()
        if(f != -1 and f != elev.floor[elev.id]):
            fsm.fsm_onFloorArrival(elev,elev.id,f)
            if (elev.behaviour[elev.id] == constants.DOOR_OPEN):
                order_OrderExecutedMsg          = Order()
                order_OrderExecutedMsg.id       = elev.id
                order_OrderExecutedMsg.floor    = f
                order_node.order_executed_publisher.publish(order_OrderExecutedMsg)

                status_msg              = Status()
                status_msg.id           = elev.id
                status_msg.behaviour    = elev.behaviour[elev.id]
                status_msg.direction    = elev.direction[elev.id]
                status_msg.floor        = elev.floor[elev.id]

                order_node.status_publisher.publish(status_msg)

        ## Timers
        if(timer.timer_timedOut()):
            timer.timer_stop()
            fsm.fsm_onDoorTimeout(elev)

            status_msg              = Status()
            status_msg.id           = elev.id
            status_msg.behaviour    = elev.behaviour[elev.id]
            status_msg.direction    = elev.direction[elev.id]
            status_msg.floor        = elev.floor[elev.id]

            order_node.status_publisher.publish(status_msg)

        for start_time in sorted(elev.unacknowledgedOrders):
            f = elev.unacknowledgedOrders[start_time][1]
            b = elev.unacknowledgedOrders[start_time][2]
            if(timer.timer_orderConfirmedTimeout(start_time)):
                order_node.get_logger().warn('Order Confirmation Timed Out!')
                fsm.fsm_onNewOrder(elev,elev.id,f,b)
                timer.timer_orderConfirmedStop(elev, start_time)

        ##Stop button
        if (fsm.driver.elev_get_stop_signal()):
            fsm.driver.elev_set_motor_direction(constants.DIRN_STOP)
            break

    order_node.get_logger().warn('Main loop done, shutting down.')
    order_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
