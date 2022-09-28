import math

import rclpy
from rclpy.node import Node

from motor_interfaces.msg import Mov, MotorState
from std_msgs.msg import Float32

from .motor import Motor

def zero(value):
    return abs(value) < 0.000001

class SmoothController(Node):
    def __init__(self):
        super().__init__('SmoothController')
        self.subscription_mov = self.create_subscription(Mov, 'mov', self.listener_callback, 10)
        self.subscription_distance = self.create_subscription(Float32, 'distance', self.set_distance, 10)
        self.publisher = self.create_publisher(MotorState, 'motorstate', 10)
        self.motor = Motor(self.get_logger())

        self.last_direction = 1
        self.last_movement_order = 0.0, 0.0
        self.distance = 2

    def listener_callback(self, msg):
        self.move(msg.forward, msg.rotate)

    def move(self, fwd, rot):
        if zero(fwd) and zero(rot):
            self.last_direction = 1
            self.motor.stop()
        else:
            if not zero(fwd):
                self.last_direction = math.copysign(1, self.last_movement_order[0])

            self.motor.set_motor_speeds(*self.circle_to_motor_speed(fwd, rot))

        self.last_movement_order = fwd, rot
        self.publish_motor_state()

    def circle_to_motor_speed(self, fwd, rot):
        if zero(rot):
            return fwd, fwd

        if zero(fwd):
            return self.last_direction * (- rot), self.last_direction * rot

        left = fwd - math.copysign(1, fwd) * rot
        right = fwd + math.copysign(1, fwd) * rot

        return min(1.0, max(-1.0, left)), min(1.0, max(-1.0, right))

    def publish_motor_state(self):
        msg = MotorState()
        msg.speed_left = float(self.motor.speed_left)
        msg.speed_right = float(self.motor.speed_right)
        self.publisher.publish(msg)

    def set_distance(self, msg):
        self.distance = msg.data
        if self.distance < 0.1 and self.motor.speed_left > 0 and self.motor.speed_right > 0:
            self.motor.stop()

def main(args=None):
    rclpy.init(args=args)

    controller = SmoothController()

    try:
        rclpy.spin(controller)
    finally:
        controller.motor.stop()

        controller.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()