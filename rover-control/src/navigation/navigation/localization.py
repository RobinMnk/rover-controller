import rclpy
from rclpy.node import Node

from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup

from motor_interfaces.msg import GPS, Waypoint, Mov, UTM
from builtin_interfaces.msg import Time

from .utility.regression import LinearRegression
from .utility.Kalman import KalmanManually as Kalman
from .utility.vehicle_model import VehicleModel

import time
import threading
import math
import numpy as np
import utm

class Localization(Node):
    def __init__(self):
        super().__init__('Localization')
        self.group = ReentrantCallbackGroup()
        self.location_publisher = self.create_publisher(Waypoint, 'location', 10)
        self.utm_publisher = self.create_publisher(UTM, 'utm', 10)

        self.gps_subscriber = self.create_subscription(GPS, 'gps', self._incoming_gps_signal, 10, callback_group=self.group)
        self.last_gps_signal = None

        self.mov_subscriber = self.create_subscription(Mov, 'mov', self._incoming_mov_order, 10, callback_group=self.group)
        self.last_mov_order = None

        self.time_subscription = self.create_subscription(Time, 'globaltime', self.check_time, 10, callback_group=self.group)
        self.last_time = conv_time(self.get_clock().now().to_msg())
        self.ready = True

        self.Hz = 10
        self.dt = 1 / self.Hz

        self._load_parameters()

        self.kalman = Kalman(self.kalman_params)
        self.regr = LinearRegression(max_history=self.regression_history)
        self.vehicle_model = VehicleModel(self.dt)
        self.position = 0., 0.
        self.orientation = 0.
        self.utm_pos = None

        thread = threading.Thread(target=self.main_loop, daemon=True)
        thread.start()

        self.get_logger().info('Localization ready')

    def _load_parameters(self):
        default_params = [
            ('kalman_P', 1000.),
            ('kalman_R', .0001),
            ('kalman_Q_dt', .1),
            ('kalman_Q_var', .13),
            ('regression_history', 4),
            ('use_regression', True),
            ('use_vehicle_simulation', False),
            ('use_kalman_filter', False)
        ]
        self.declare_parameters(
            namespace='',
            parameters=default_params
        )
        load_param = lambda param: self.get_parameter(param).get_parameter_value()

        self.kalman_params = { param: load_param(param).double_value for param, _ in default_params if param.startswith("kalman")}
        self.regression_history = load_param('regression_history').integer_value
        self.use_regression = load_param('use_regression').bool_value
        self.use_vehicle_simulation = load_param('use_vehicle_simulation').bool_value
        self.use_kalman_filter = load_param('use_kalman_filter').bool_value

    def main_loop(self):
        while rclpy.ok():
            if self.ready:
                self.ready = False
                self.update_position()
                self.location_publisher.publish(self._toWaypoint())
                if self.utm_pos is not None:
                    self.utm_publisher.publish(self._toUTM())
                time.sleep(0.01)

    def update_position(self):
        if self.last_gps_signal is not None:
            _, gps_prediction = self.last_gps_signal

            self.utm_pos = utm.from_latlon(gps_prediction.lat, gps_prediction.lon)

            update = [self.utm_pos[0], self.utm_pos[1]]
            new_position = self.kalman.apply(update) if self.use_kalman_filter else update

            if self.use_regression:
                self.regr.add_point(*new_position)
                self.orientation = self.regr.angle()
            else:
                self.orientation = angle_between(self.position, new_position)

            self.position = new_position
            self.last_gps_signal = None

        elif self.use_vehicle_simulation and self.last_mov_order is not None:
            _, mov = self.last_mov_order
            self.position, x = self.vehicle_model.simulate(self.position, self.orientation, [mov.forward, mov.rotate])
            self.orientation = normalize_angle(x)

    def _incoming_gps_signal(self, msg):
        # comes in at rate of 1 Hz
        if msg.lat != 0 and msg.lon != 0:
            self.last_gps_signal = (self.get_clock().now(), msg)

            if not self.kalman.ready:
                self.kalman.setup(msg.lat, msg.lon)
                self.get_logger().info('Localization initialized at\nlat: {}\nlon: {}'.format(msg.lat, msg.lon))

    def _incoming_mov_order(self, msg):
        # comes in at rate of 8 Hz if navigator is running, otherwise irregular
        self.last_mov_order = (self.get_clock().now(), msg)

        if abs(msg.rotate) > math.pi / 16:
            # driving a curve, reset the regression
            self.regr.reset(3)

    def _toWaypoint(self):
        x, y = self.position
        pos = Waypoint()
        pos.x = float(x)
        pos.y = float(y)
        pos.theta = float(self.orientation)
        return pos

    def _toUTM(self):
        utm = UTM()
        utm.x = float(self.utm_pos[0])
        utm.y = float(self.utm_pos[1])
        utm.zone_nr = int(self.utm_pos[2])
        utm.zone_str = self.utm_pos[3]
        utm.angle = float(self.orientation)
        return utm

    def check_time(self, current_time):
        diff = conv_time(current_time) - self.last_time
        if diff > self.dt:
            self.last_time += self.dt
            self.ready = True

def conv_time(current_time):
    return current_time.sec + current_time.nanosec * 1e-9

def angle_between(v1, v2):
    return math.atan2(v2[1] - v1[1], v2[0] - v1[0])

def normalize_angle(x):
    while x > 2 * math.pi:
        x -= 2 * math.pi
    while x < 0:
        x += 2 * math.pi
    return x

def vector_to_angle(v): return math.atan2(v[1], v[0])

def angle_to_vector(a): return np.array([math.cos(a), math.sin(a)])

def vector_add(v1, v2):
    return [v1[i] + v2[i] for i in range(len(v1))]

def main(args=None):
    rclpy.init(args=args)

    try:
        localization = Localization()
        executor = MultiThreadedExecutor(num_threads=4)
        executor.add_node(localization)

        try:
            executor.spin()
        finally:
            executor.shutdown()
            localization.destroy_node()

    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()