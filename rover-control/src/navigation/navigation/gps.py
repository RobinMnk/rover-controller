import rclpy
from rclpy.node import Node

from motor_interfaces.msg import GPS

from gps3 import gps3

class GPS_Module(Node):
    def __init__(self):
        super().__init__('GPS_Module')
        self.publisher = self.create_publisher(GPS, 'gps', 10)

        self.data_stream = gps3.DataStream()
        self.gpsd_socket = gps3.GPSDSocket()
        self.gpsd_socket.connect()
        self.gpsd_socket.watch()

        self.update()

    def update(self):
        for new_data in self.gpsd_socket:
            if new_data:
                self.data_stream.unpack(new_data)

                msg = GPS()
                msg.lat = self.extract('lat')
                msg.lon = self.extract('lon')
                msg.epx = self.extract('epx')
                msg.epy = self.extract('epy')
                self.publisher.publish(msg)

    def extract(self, key):
        value = self.data_stream.TPV[key]
        return float(value) if value != "n/a" else 0.0
    
def main(args=None):
    rclpy.init(args=args)

    gps_module = GPS_Module()

    rclpy.spin(gps_module)

    gps_module.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()