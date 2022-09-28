import rclpy
from rclpy.node import Node

from builtin_interfaces.msg import Time

import threading

class GlobalTimer(Node):
    def __init__(self):
        super().__init__('GlobalTimer')
        self.time_publisher = self.create_publisher(Time, 'globaltime', 10)

        thread = threading.Thread(target=rclpy.spin, args=(self,), daemon=True)
        thread.start()
        self.Hz = 20
        self.rate = self.create_rate(self.Hz, self.get_clock())

        self.time = Time()

        self.navigation_main_loop()

    def navigation_main_loop(self):
        while rclpy.ok():
            stamp = self.get_clock().now().to_msg()
            self.time.sec = stamp.sec
            self.time.nanosec = stamp.nanosec
            self.time_publisher.publish(self.time)
            self.rate.sleep()


def main(args=None):
    rclpy.init(args=args)

    globaltime = GlobalTimer()

    rclpy.spin(globaltime)

    globaltime.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()