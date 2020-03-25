# Copyright 2016 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from ros2_msg.msg import Order

class OrderPublisher(Node):

    def __init__(self):
        super().__init__('order_publisher')
        #self.publisher_ = self.create_publisher(String, 'topic', 10)
        self.pubber = self.create_publisher(Order, 'orders', 10)
        period = 0.5  # seconds
        #self.timer = self.create_timer(period, self.callback)
        #self.i = 0

    #def callback(self):
        # msg = MyMsg()
        # #msg.data = 'Hello World: %d' % self.i
        # #self.publisher_.publish(msg)
        # msg.id = self.i
        # msg.tekst = "Gutta Gutta Gutta! H er greit"
        # self.pubber.publish(msg)
        # self.get_logger().info('Publishing: "%s"' % msg.tekst)
        # self.i += 1


def main(args=None):
    rclpy.init(args=args)

    order_publisher = OrderPublisher()
    while(1):
        rclpy.spin_once(order_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    order_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
