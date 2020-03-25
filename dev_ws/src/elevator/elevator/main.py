
#### Add script path for ros compiled files to find them ###
install_dir = os.path.join(os.path.dirname(__file__))
sys.path.append(os.path.abspath(install_dir))
############################################################


#~ python libs
import copy
import numpy as np
import os
import rclpy
import socket
import sys
import time

#~ ROS packages
from rclpy.node   import Node
from ros2_msg.msg import Order
from ros2_msg.msg import OrderConfirmed
from ros2_msg.msg import Status

#~ source files
from constants      import *
from distributor    import *
from fsm            import *
from messages       import *
from timer          import *
from status         import LocalElevator
from statusCopy     import SingleElevatorCopy


## Get IP for this computer ##
s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
s.connect(("8.8.8.8", 80))
local_id = s.getsockname()[0][-3:] #last 3 characters
local_id = local_id.replace('.','')
###########################

elev = LocalElevator(int(local_id))

#~~~~=*=~~~~# ROS #~~~~=*=~~~~#

class ElevatorNode(Node):

    def __init__(self):
        super().__init__('order_node_' + str(local_id))

        #~Subscribers for listening to topics
        self.order_subscriber           = self.create_subscription(Order, 'orders', self.order_callback, 10)
        self.order_executed_subscriber  = self.create_subscription(Order, 'executed_orders', self.order_executed_callback, 10)
        self.order_confirmed_subscriber = self.create_subscription(Order, 'confirmed_orders', self.order_confirmed_callback, 10)
        self.init_subscriber            = self.create_subscription(Status, 'init', self.init_callback, 10)
        self.node_subscriber            = self.create_subscription(Status, 'node', self.node_callback, 10)
        self.status_subscriber          = self.create_subscription(Status, 'status', self.status_callback, 10)

        #~Publishers for sending to topics
        self.order_publisher            = self.create_publisher(Order, 'orders', 10)
        self.order_executed_publisher   = self.create_publisher(Order, 'executed_orders', 10)
        self.order_confirmed_publisher  = self.create_publisher(Order, 'confirmed_orders', 10)
        self.init_publisher             = self.create_publisher(Status, 'init', 10)
        self.node_publisher             = self.create_publisher(Status, 'node', 10)
        self.status_publisher           = self.create_publisher(Status, 'status', 10)

        return

  #~~~ Callback functions ~~~#
    def node_callback(self,msg):
        elev.floor[msg.id]      = msg.floor
        elev.behaviour[msg.id]  = msg.behaviour
        elev.direction[msg.id]  = msg.direction

        if (msg.id not in elev.queue):  # create empty queue
            self.get_logger().warn('Node from id %d received\n' %(msg.id))
            matrix = [[0 for b in range(N_BUTTONS)] for f in range(N_FLOORS)]
            elev.queue[msg.id] = matrix

        for f in range(N_FLOORS):
            for b in range(N_BUTTONS):
                index = f*N_BUTTONS + b
                elev.queue[msg.id][f][b] = msg.queue[index]

        return

    def init_callback(self,msg):
        # add the new elevator to local node
        elev.floor[msg.id]      = msg.floor
        elev.behaviour[msg.id]  = msg.behaviour
        elev.direction[msg.id]  = msg.direction
        elev.queue[msg.id]      = [[0 for b in range(N_BUTTONS)] for f in range(N_FLOORS)]
        self.get_logger().warn('Init from id %d received!\n' %(msg.id))

        # answer new elevator with info about local node
        node_msg = messages_createMessage(elev, MSG_NODE)
        self.node_publisher.publish(node_msg)

        return


    def status_callback(self, msg):
        elev.floor[msg.id]      = msg.floor
        elev.behaviour[msg.id]  = msg.behaviour
        elev.direction[msg.id]  = msg.direction

        return

    def order_confirmed_callback(self, msg):
        for (start_time in sorted(elev.unacknowledgedOrders)):
            id  = elev.unacknowledgedOrders[start_time][0]
            f   = elev.unacknowledgedOrders[start_time][1]
            b   = elev.unacknowledgedOrders[start_time][2]

            if (msg.id == id and msg.floor == f and msg.button == b):
                timer_orderConfirmedStop(elev,start_time)
                fsm.fsm_onNewOrder(elev,id,f,b)

        #Print all queues
        self.get_logger().warn("Printing all queues")
        for id in sorted(elev.queue):
            print("Id: %d\t" %(id), end = " ")
            print(elev.queue[id])
        print(" --- \n")

        return

    def order_executed_callback(self, msg):
        self.get_logger().warn('Order executed at ID: %d, Floor: %d, Button: %s\n' %(msg.id, msg.floor, msg.button))
        if (msg.id != elev.id):
            fsm.fsm_onFloorArrival(elev, msg.id, msg.floor)

        self.get_logger().warn("Printing all queues")
        for id in sorted(elev.queue):
            print("Id: %d\t" %(id), end = " ")
            print(elev.queue[id])
        print(" --- \n")

        return

    def order_callback(self, msg):
        self.get_logger().info('New order from: %d, Floor: %d, Button: %s\n' %(msg.id, msg.floor, msg.button))

        if (msg.button == BTN_CAB):
            fsm.fsm_onNewOrder(elev,msg.id,msg.floor,msg.button)
            self.get_logger().info('New cab order distributed to: %s\n' %(msg.id))
            return

        else:
#################### ORDER DISTRIBUTER ASSIGNS NEW ORDER #########################
            min_id = 0          # ID of elevator with the least cost
            min_duration = 999  # Time duration of elev with least cost

            elev_copy = copy.deepcopy(elev)

            for id in sorted(elev_copy.queue):
                f   = elev_copy.floor[id]
                qu  = elev_copy.queue[id]
                bh  = elev_copy.behaviour[id]
                dir = elev_copy.direction[id]

                single_elev_copy = SingleElevatorCopy(id, f, bh, dir, qu)
                single_elev_copy.queue[id][msg.floor][msg.button] = 1
                duration = distributor_timeToIdle(single_elev_copy)
                self.get_logger().warn('ID: %s  | Duration: %d\n' %(id, duration))

                if (duration < min_duration):
                    min_id = id
                    min_duration = duration
###################################################################################

            self.get_logger().warn('New hall order distributed to: %s\n' %(min_id))

            if (elev.id == min_id):
                fsm.fsm_onNewOrder(elev, min_id, msg.floor,msg.button)
                order_confirmed_msg = messages_createMessage(elev, MSG_ORDER_CONFIRMED, msg.floor, msg.button)
                self.order_confirmed_publisher.publish(order_confirmed_msg)

                if (elev.behaviour[elev.id] == DOOR_OPEN and elev.floor[elev.id] == msg.floor):
                    order_OrderExecutedMsg = messages_createMessage(elev, MSG_ORDER_EXECUTED)
                    self.order_executed_publisher.publish(order_OrderExecutedMsg)

                status_msg = messages_createMessage(elev, MSG_STATUS)
                self.status_publisher.publish(status_msg)

            else:
                #Add to unacknowledgedOrders
                timer_orderConfirmedStart(elev, min_id, msg.floor, msg.button)
        return


############### MAIN LOOP #####################
def main(args=None):
    global elev

    rclpy.init(args=args)
    order_node = ElevatorNode()

    fsm.fsm_init(elev)
    order_node.get_logger().warn('Init complete! ID: %d\n' %(elev.id))

    init_msg = messages_createMessage(elev, MSG_INIT)
    order_node.init_publisher.publish(init_msg)

    rclpy.spin_once(order_node,executor=None,timeout_sec=0)

    order_node.get_logger().warn("Init message sent from self!")

    while (rclpy.ok()):
        rclpy.spin_once(order_node,executor=None,timeout_sec=0)

        ## Request button
        for f in range(N_FLOORS):
            for b in range(N_BUTTONS):
                v = fsm.driver.elev_get_button_signal(b, f)
                if (v and (v != elev.queue[elev.id][f][b])):
                    while (fsm.driver.elev_get_button_signal(b, f)):
                        pass

                    order_newOrderMsg = messages_createMessage(elev, MSG_NEW_ORDER, f, b)
                    order_node.order_publisher.publish(order_newOrderMsg)

        ## Floor sensor
        f = fsm.driver.elev_get_floor_sensor_signal()
        if ((f != -1) and (f != elev.floor[elev.id])):
            fsm.fsm_onFloorArrival(elev, elev.id, f)

            if (elev.behaviour[elev.id] == DOOR_OPEN):
                order_OrderExecutedMsg = messages_createMessage(elev, MSG_ORDER_EXECUTED)
                order_node.order_executed_publisher.publish(order_OrderExecutedMsg)
                status_msg = messages_createMessage(elev, MSG_STATUS)
                order_node.status_publisher.publish(status_msg)

        ## Timers
        if (timer_timedOut()):
            timer_stop()
            fsm.fsm_onDoorTimeout(elev)

            status_msg = messages_createMessage(elev, MSG_STATUS)
            order_node.status_publisher.publish(status_msg)

        for start_time in sorted(elev.unacknowledgedOrders):
            f = elev.unacknowledgedOrders[start_time][1]
            b = elev.unacknowledgedOrders[start_time][2]
            if (timer_orderConfirmedTimeout(start_time)):
                order_node.get_logger().error('Order Confirmation Timed Out!')
                fsm.fsm_onNewOrder(elev,elev.id,f,b)
                timer_orderConfirmedStop(elev, start_time)

        ##Stop button
        if (fsm.driver.elev_get_stop_signal()):
            fsm.driver.elev_set_motor_direction(DIRN_STOP)
            break

    order_node.get_logger().error('Main loop done, shutting down.')
    order_node.destroy_node()
    rclpy.shutdown()

    return

if __name__ == '__main__':
    main()
