#~ python libs
import copy
import numpy as np
import os
import rclpy
import socket
import sys
import time
from ctypes import *

#### Add script path for ros compiled files to find them ###
install_dir = os.path.join(os.path.dirname(__file__))
sys.path.append(os.path.abspath(install_dir))
############################################################

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
from status         import LocalElevator
from statusCopy     import SingleElevatorCopy


#~~~ Include driver ~~~#
driver_path = os.path.join(os.path.dirname(__file__), '../../../../../../src/elevator/elevator/driver/driver.so')
driver = cdll.LoadLibrary(os.path.abspath(driver_path))
driver.elev_init(ELEV_MODE) #Simulator / Physical model

## Get IP for this computer ##
try:
    local_ip = util_getLocalIp()
    local_id = local_ip.getsockname()[0][-3:] #last 3 characters
    local_id = local_id.replace('.','')
except:
    print("NO INTERNET!")
    print("NO ELEVATOR TODAY!")
    print("SHUTTING DOWN!")
    exit()
###########################

elev = LocalElevator(int(local_id))

#~~~~~~~~# ROS #~~~~~~~~#

class ElevatorNode(Node):

    def __init__(self):
        super().__init__('elev_node_'+str(local_id))

        #~ Creating subscribers for listening to topics
        self.init_subscriber            = self.create_subscription(Init, 'init', self.init_callback, 10)
        self.node_subscriber            = self.create_subscription(NodeMsg, 'node', self.node_callback, 10)
        self.status_subscriber          = self.create_subscription(Status, 'status', self.status_callback, 10)
        self.order_subscriber           = self.create_subscription(Order, 'orders', self.order_callback, 10)
        self.order_executed_subscriber  = self.create_subscription(OrderExecuted, 'executed_orders', self.order_executed_callback, 10)
        self.order_confirmed_subscriber = self.create_subscription(OrderConfirmed, 'confirmed_orders', self.order_confirmed_callback, 10)
    #    self.order_heartbeat_subscriber = self.create_subscription(Heartbeat, 'heartbeat', self.heartbeat_callback,10)

        #~ Creating publishers for sending to topics
        self.init_publisher             = self.create_publisher(Init, 'init', 10)
        self.node_publisher             = self.create_publisher(NodeMsg, 'node', 10)
        self.status_publisher           = self.create_publisher(Status, 'status', 10)
        self.order_publisher            = self.create_publisher(Order, 'orders', 10)
        self.order_executed_publisher   = self.create_publisher(OrderExecuted, 'executed_orders', 10)
        self.order_confirmed_publisher  = self.create_publisher(OrderConfirmed, 'confirmed_orders', 10)
        self.heartbeat_publisher        = self.create_publisher(Heartbeat, 'heartbeat', 10)

        return

  #~~~ Callback functions ~~~#

    # def heartbeat_callback(self,msg):
    #     elev.heartbeat[msg.id] = get time now;
    # legg til at heisen funker , typ status ellerno

    #~ Callback for when an elevator initializes
    def init_callback(self, msg):
        if (msg.id == elev.id):
            return

        self.get_logger().warn('Init from id %d received!\n' %(msg.id))
        msg_update_elev(elev, msg)

        if (msg.initmode == RECONNECT) or (msg.id not in elev.queue):
                elev.queue[msg.id]  = [[0 for btn in range(N_BUTTONS)] for flr in range(N_FLOORS)]
        nodeMsg = msg_create_nodeMessage(elev, msg.id, msg.initmode)
        self.node_publisher.publish(nodeMsg)

        return

    #~ Callback for when an elevator responds to a new initialization
    def node_callback(self, msg):
        if (msg.id == elev.id):
            return

        self.get_logger().warn('Node from id %d received!\n' %(msg.id))
        msg_update_elev(elev, msg)

        if (msg.id not in elev.queue):
            elev.queue[msg.id] = [[0 for btn in range(N_BUTTONS)] for flr in range(N_FLOORS)]

        for floor in range(N_FLOORS):
            for btn in range(N_BUTTONS):
                #Mapping 1D -> 2D array
                elev.queue[msg.id][floor][btn] = msg.queue[floor*N_BUTTONS + btn]

            #checking if initialized elevator should retrieve its old cab-queue:
            if ((msg.initmode == RESTART) and (msg.cabqueue[floor] == 1) and (msg.knownid == elev.id)):
                fsm_onNewOrder(elev, elev.id, floor, BTN_CAB)

        return

    #~ Callback for when an elevator updates its state
    def status_callback(self, msg):
        if (msg.id == elev.id):
            return

        msg_update_elev(elev, msg)

        if (msg.network == OFFLINE):
            for floor in range(N_FLOORS):
                for btn in range(N_BUTTONS):
                    if (btn == BTN_CAB or elev.queue[msg.id][floor][btn] == 0):
                        continue

                    newOrderMsg = msg_create_newOrderMessage(elev, floor, btn)
                    self.order_publisher.publish(newOrderMsg)
                    elev.queue[msg.id][floor][btn] = 0

        return

    #~ Callback for when an elevator confirms an order
    def order_confirmed_callback(self, msg):
        for start_time in sorted(elev.unacknowledgedOrders):
            id      = elev.unacknowledgedOrders[start_time][0]
            floor   = elev.unacknowledgedOrders[start_time][1]
            btn     = elev.unacknowledgedOrders[start_time][2]

            if (msg.id == id and msg.floor == floor and msg.button == btn):
                timer_orderConfirmedStop(elev, start_time)
                fsm_onNewOrder(elev, id, floor, btn)
        return

    #~ Callback for when an elevator successfully executes an order
    def order_executed_callback(self, msg):
        self.get_logger().warn('Order executed at ID: %d, Floor: %d' %(msg.id, msg.floor))
        if (msg.id != elev.id):
            fsm_onFloorArrival(elev, msg.id, msg.floor)
        return

    #~ Callback for when an elevator has a new order
    def order_callback(self, msg):
        self.get_logger().info('New order from: %d, Floor: %d, Button: %s\n' %(msg.id, msg.floor, msg.button))

        if (msg.button == BTN_CAB):
            fsm_onNewOrder(elev,msg.id,msg.floor,msg.button)
            self.get_logger().info('New cab order distributed to: %s\n' %(msg.id))
            return

        else:
#################### ORDER DISTRIBUTER ASSIGNS NEW ORDER #########################
            min_id          = elev.id   # ID of elevator with the least cost
            min_duration    = 999       # Time duration of elev with least cost

            elev_copy = copy.deepcopy(elev)

            for id in sorted(elev_copy.queue):
                if (elev_copy.network[id] == OFFLINE):
                    continue
                floor       = elev_copy.floor[id]
                queue       = elev_copy.queue[id]
                dir         = elev_copy.direction[id]
                behaviour   = elev_copy.behaviour[id]

                single_elev_copy = SingleElevatorCopy(id, floor, behaviour, dir, queue)
                single_elev_copy.queue[id][msg.floor][msg.button] = 1
                duration = distributor_timeToIdle(single_elev_copy)

                if (duration < min_duration):
                    min_id = id
                    min_duration = duration
###################################################################################

            self.get_logger().warn('New hall order distributed to: %s\n' %(min_id))

            if (elev.id == min_id):
                fsm_onNewOrder(elev, min_id, msg.floor,msg.button)
                orderConfirmedMsg = msg_create_orderConfirmedMessage(elev, msg.floor, msg.button)
                self.order_confirmed_publisher.publish(orderConfirmedMsg)

                if (elev.behaviour[elev.id] == DOOR_OPEN and elev.floor[elev.id] == msg.floor):
                    orderExecutedMsg = msg_create_orderExecutedMessage(elev)
                    self.order_executed_publisher.publish(orderExecutedMsg)

                statusMsg = msg_create_statusMessage(elev)
                self.status_publisher.publish(statusMsg)

            else:
                #Add to unacknowledgedOrders
                timer_orderConfirmedStart(elev, min_id, msg.floor, msg.button)
        return



#~~~~~~~~# Main function #~~~~~~~~#
def main(args=None):
    global elev
    try:
        Elevator.destroy_node()
        rclpy.shutdown()
    except:
        print('ORDER NOT PURGED')
        pass

    rclpy.init(args=args)
    Elevator = ElevatorNode()

    fsm_init(elev)
    initMsg = msg_create_initMessage(elev, RESTART)
    Elevator.init_publisher.publish(initMsg)

    rclpy.spin_once(Elevator, executor=None, timeout_sec=5)
    Elevator.get_logger().warn("Init message sent from self!")

    timer_heartbeatStart()

    while (rclpy.ok()):
        rclpy.spin_once(Elevator, executor=None, timeout_sec=0)

        #~ Check for new button push
        for floor in range(N_FLOORS):
            for btn in range(N_BUTTONS):
                if (events_newButtonPush(elev, floor, btn)):
                    if (elev.network[elev.id] == OFFLINE and btn == BTN_CAB):
                        fsm_onNewOrder(elev, elev.id, floor, btn)
                        continue
                    newOrderMsg = msg_create_newOrderMessage(elev, floor, btn)
                    Elevator.order_publisher.publish(newOrderMsg)

        #~ Check floor sensors
        if (events_onNewFloor(elev)):
            fsm_onFloorArrival(elev, elev.id, driver.elev_get_floor_sensor_signal())
            if (elev.behaviour[elev.id] == DOOR_OPEN):
                orderExecutedMsg  = msg_create_orderExecutedMessage(elev)
                statusMsg               = msg_create_statusMessage(elev)
                Elevator.order_executed_publisher.publish(orderExecutedMsg)
                Elevator.status_publisher.publish(statusMsg)

        #~ Doors open-timer timeout
        if (timer_doorsTimeout()):
            timer_doorsStop()

            fsm_onDoorTimeout(elev)

            statusMsg = msg_create_statusMessage(elev)
            Elevator.status_publisher.publish(statusMsg)

        #~ Mechanical error-timer timeout
        if (timer_executionTimeout()):
            Elevator.get_logger().error('Mechanical error!')
            timer_executionStop()

            elev.network[elev.id] = OFFLINE
            statusMsg  = msg_create_statusMessage(elev)
            Elevator.status_publisher.publish(statusMsg)

            fsm_onMechanicalError(elev)

            initMsg    = msg_create_initMessage(elev, RECONNECT)
            Elevator.init_publisher.publish(initMsg)

        #~ Checking timeouts on the unconfirmed orders
        for start_time in sorted(elev.unacknowledgedOrders):
            if (timer_orderConfirmedTimeout(start_time)):
                Elevator.get_logger().error('Order Confirmation Timed Out!')
                floor   = elev.unacknowledgedOrders[start_time][1]
                btn     = elev.unacknowledgedOrders[start_time][2]
                fsm_onNewOrder(elev, elev.id, floor, btn)
                timer_orderConfirmedStop(elev, start_time)

        #~ Send heartbeat
        if (elev.network[elev.id] == ONLINE and timer_heartbeatTimeout()):
            timer_heartbeatStop()
            heartbeatMsg = msg_create_heartbeatMessage(elev)
            Elevator.heartbeat_publisher.publish(heartbeatMsg)
            timer_heartbeatStart()

        #~ Heartbeat timeout
        # for start_time in sorted(elev.hearBeats_soimethingeifngngferjingriufv):
        #     if (timer_orderConfirmedTimeout(start_time)): # Endre til at heartbeat er for treg
        #         Elevator.get_logger().error('Order Confirmation Timed Out!')
        #         floor = elev.unacknowledgedOrders[start_time][1]
        #         btn = elev.unacknowledgedOrders[start_time][2]
        #         fsm_onNewOrder(elev,elev.id,floor,btn)
        #         timer_orderConfirmedStop(elev, start_time)



        #~ Check if elevator has lost power or network connection
        # try:
        #     #~ Throws an exception if the elevator can not connect to network
        #     local_ip = util_getLocalIp()
        #     if (events_offlineAndNoOrders(elev)):
        #         elev.network[elev.id] = ONLINE
        #         initMsg = msg_create_initMessage(elev, RECONNECT)
        #         time.sleep(0.5)
        #         print('WE ARE NOW ONLINE')
        #         Elevator.init_publisher.publish(initMsg)
        #     else:
        #         for id in sorted(elev.queue):
        #             if (events_otherElevOffline(elev,id)):
        #                 print('OTHER ELEVATOR IS OFFLINE:    %d'%id)
        #                 elev.network[id] = OFFLINE
        #                 for f in range(N_FLOORS):
        #                     for b in range(N_BUTTONS):
        #                         if (b == BTN_CAB or elev.queue[id][f][b] == 0):
        #                             continue
        #                             newOrderMsg = messages_createMessage(elev, MSG_NEW_ORDER, f, b)
        #                             Elevator.order_publisher.publish(newOrderMsg)
        #                             elev.queue[id][f][b] = 0
        # except:
        #     #print("THIS ELEVATOR IS NOW OFFLINE!")
        #     elev.network[elev.id] = OFFLINE
        #     for f in range(N_FLOORS):
        #         for b in range(N_BUTTONS):
        #             if (b == BTN_CAB):
        #                 continue
        #             elev.queue[elev.id][f][b] = 0

        #~~~ Check stop button ~~~#
        if (driver.elev_get_stop_signal()):
            driver.elev_set_motor_direction(DIRN_STOP)
            break

    Elevator.get_logger().error('Main loop done, shutting down.')
    Elevator.destroy_node()
    rclpy.shutdown()

    return

if __name__ == '__main__':
    main()
