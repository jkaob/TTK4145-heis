import sys
sys.path.append("/home/student/Desktop/Gruppe1_Sanntid/TTK4145-heis/dev_ws/install/elevator/lib/python3.6/site-packages/elevator")
import timer
import fsm
import constants
from status import LocalElevator
import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from ros2_msg.msg import Order

elev = LocalElevator()


#######ROS########
# For å sende en ordre må callback kjøre fsm_onNewOrder(msg.id,msg.floor,msg.button)
class OrderSubscriber(Node):

    def __init__(self):
        super().__init__('order_subscriber')
        self.subscription = self.create_subscription(Order,'orders' ,self.order_callback,10)

    def order_callback(self, msg):
        self.get_logger().info('New order!\n Id: %d\n Floor: %d\n Button: %s\n' %(msg.id, msg.floor, msg.button))
        #fsm.fsm_onNewOrder(msg.id, msg.floor, msg.button)




###########################
def main(args=None):
    rclpy.init(args=args)
    order_subscriber = OrderSubscriber()

    global elev
    fsm.fsm_init(elev)

    while(rclpy.ok()):
        #order_subscriber.get_logger().info('Her')
        rclpy.spin_once(order_subscriber,executor=None,timeout_sec=0)

        #rclpyget_logger().info('Forbi')
        ## Request button
        for f in range(constants.N_FLOORS):
            for b in range(constants.N_BUTTONS):
                v = fsm.driver.elev_get_button_signal(b,f)
                if(v and (v != elev.queue[elev.id][f][b])):
                    fsm.fsm_onNewOrder(elev,f,b)

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


    order_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
