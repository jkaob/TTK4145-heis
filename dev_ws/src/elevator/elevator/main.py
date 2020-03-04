import sys
sys.path.append("/home/student/Desktop/Gruppe1_Sanntid/TTK4145-heis/dev_ws/install/elevator/lib/python3.6/site-packages/elevator")
import timer
import fsm
import constants
import socket
import os
import rclpy
import distributor
from status import LocalElevator
from rclpy.node import Node
from std_msgs.msg import String
from ros2_msg.msg import Order

s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
s.connect(("8.8.8.8", 80))
local_id = s.getsockname()[0][-3:]

elev = LocalElevator(local_id)


#######ROS########
# For å sende en ordre må callback kjøre fsm_onNewOrder(msg.id,msg.floor,msg.button)
class OrderSubscriber(Node):

    def __init__(self):
        super().__init__('order_subscriber')
        self.subscription = self.create_subscription(Order,'orders' ,self.order_callback,10)

    def order_callback(self, msg):
        self.get_logger().info('New order!\n Id: %d\n Floor: %d\n Button: %s\n' %(msg.id, msg.floor, msg.button))


#################### ORDER DISTRIBUTER ASSIGNS NEW ORDER #########################
        min_duration = 999
        min_id = 0

        for id in sorted(elev.queue):
            elev_copy = LocalElevator(id,elev.floor[id],elev.behaviour[id],elev.direction[id],elev.queue[id])
            duration = distributor.distributor_timeToIdle(elev_copy)
            if(duration < min_duration):
                min_duration = duration
                min_id = id
        self.get_logger().info('New order distributed to: %s\n' %(min_id))
###################################################################################
        if (elev.id == min_id):
            fsm.fsm_onNewOrder(elev, msg.id, msg.floor,msg.button)
            ##publish order received to reset timer
        else:
            #Start confirmation timer
            # if time_out - rerun order_callback(msg)

############### MAIN LOOP #####################
def main(args=None):
    rclpy.init(args=args)
    order_subscriber = OrderSubscriber()

    global elev
    fsm.fsm_init(elev)
    order_subscriber.get_logger().info('Id: %s' %(elev.id))
    while(rclpy.ok()):
        #order_subscriber.get_logger().info('Her')
        rclpy.spin_once(order_subscriber,executor=None,timeout_sec=0)

        #order_subscriber.get_logger().info('Forbi')
        ## Request button
        for f in range(constants.N_FLOORS):
            for b in range(constants.N_BUTTONS):
                v = fsm.driver.elev_get_button_signal(b,f)
                if(v and (v != elev.queue[elev.id][f][b])):
                    fsm.fsm_onNewOrder(elev,elev.id,f,b)

        ## Floor sensor
        f = fsm.driver.elev_get_floor_sensor_signal()
        if(f != -1 and f != elev.floor):
            fsm.fsm_onFloorArrival(elev,f)

        ## Timer
        if(timer.timer_timedOut()):
            timer.timer_stop()
            fsm.fsm_onDoorTimeout(elev)


        ##Stop button
        if (fsm.driver.elev_get_stop_signal()):
            fsm.driver.elev_set_motor_direction(constants.DIRN_STOP)
            break


    order_subscriber.get_logger().warn('DONE!')
    order_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
