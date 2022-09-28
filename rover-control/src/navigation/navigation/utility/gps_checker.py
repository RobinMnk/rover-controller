import math

import rclpy
from rclpy.node import Node

from motor_interfaces.msg import GPS

class GPS_Checker(Node):
    def __init__(self):
        super().__init__('GPS_Checker')
        self.subscription_mov = self.create_subscription(GPS, 'gps', self.listener_callback, 10)
        self.observations = list()
        self.max_dist = 0.0

    def listener_callback(self, msg):
        ix = max(0, len(self.observations) - 200)
        for obs in self.observations[ix:]:
            dist = abs(haversine(*obs, msg.lat, msg.lon))
            if dist > self.max_dist:
                self.max_dist = dist
                self.get_logger().info("New Jump - max dist:  {}".format(self.max_dist))

        self.observations.append( (msg.lat, msg.lon) )

def haversine(lon1, lat1, lon2, lat2):
    """
    Calculate the great circle distance between two points
    on the earth (specified in decimal degrees)
    """
    # convert decimal degrees to radians
    lon1, lat1, lon2, lat2 = list(map(math.radians, [lon1, lat1, lon2, lat2]))

    # haversine formula
    dlon = lon2 - lon1
    dlat = lat2 - lat1
    a = math.sin(dlat / 2)**2 + math.cos(lat1) * math.cos(lat2) * math.sin(dlon / 2)**2
    c = 2 * math.asin(math.sqrt(a))
    # 6367 km is the radius of the Earth
    km = 6367 * c
    meter = km * 1000
    return meter

def main(args=None):
    rclpy.init(args=args)

    controller = OdometryConverter()

    rclpy.spin(controller)

    controller.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()