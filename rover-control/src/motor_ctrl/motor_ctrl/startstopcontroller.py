import math

import rclpy
from rclpy.node import Node

from motor_interfaces.msg import Mov

from .motor import Motor

class StartStopController(Node):
    def __init__(self):
        super().__init__('StartStopController')
        self.subscription = self.create_subscription(Mov, 'mov', self.listener_callback, 10)
        self.motor = Motor(self.get_logger())

    def listener_callback(self, msg):
        self.get_logger().info('I heard the movement Order: fwd: %f\trotate: %f' % (msg.forward, msg.rotate))

        fwd, rot = msg.forward, msg.rotate
        if fwd != 0:
            self.motor.drive(100 * abs(fwd), math.copysign(1, fwd))
        elif rot != 0:
            self.motor.turn(100 * rot)
        else:
            self.motor.stop()

def main(args=None):
    rclpy.init(args=args)

    controller = StartStopController()

    rclpy.spin(controller)

    controller.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()