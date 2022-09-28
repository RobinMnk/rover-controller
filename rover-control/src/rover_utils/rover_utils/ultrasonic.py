import rclpy
from rclpy.node import Node

from std_msgs.msg import Float32

import RPi.GPIO as GPIO
import time

Tr = 11
Ec = 8

class UltrasonicDistance(Node):

    def __init__(self):
        super().__init__('UltrasonicDistance')
        self.distance_pub = self.create_publisher(Float32, 'distance', 10)
        timer_period = 0.1
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        msg = Float32()
        dist = self.get_distance()
        msg.data = dist
        self.distance_pub.publish(msg)

    def get_distance(self):
        GPIO.setwarnings(False)
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(Tr, GPIO.OUT, initial=GPIO.LOW)
        GPIO.setup(Ec, GPIO.IN)
        GPIO.output(Tr, GPIO.HIGH)
        time.sleep(0.000015)
        GPIO.output(Tr, GPIO.LOW)
        while not GPIO.input(Ec):
            pass
        t1 = time.time()
        while GPIO.input(Ec):
            pass
        t2 = time.time()
        return (t2 - t1) * 340 / 2


def main(args=None):
    rclpy.init(args=args)

    ultrasonice_distance = UltrasonicDistance()

    rclpy.spin(ultrasonice_distance)

    ultrasonice_distance.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()