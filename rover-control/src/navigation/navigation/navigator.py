import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node

from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup

from motor_interfaces.action import DriveToLocation
from motor_interfaces.msg import Mov, Waypoint
from builtin_interfaces.msg import Time

import time
import math

class Navigator(Node):
    def __init__(self):
        super().__init__('Navigator')
        self.group = ReentrantCallbackGroup()
        self._action_server = ActionServer(
            self,
            DriveToLocation,
            'drive_to_location',
            self.execute_callback,
            callback_group=self.group
        )

        self.movement_order_publisher = self.create_publisher(Mov, 'mov', 10)

        self.location_subscription = self.create_subscription(Waypoint, 'location', self.incoming_position_update, 10, callback_group=self.group)
        self.time_subscription = self.create_subscription(Time, 'globaltime', self.check_time, 10, callback_group=self.group)
        self.last_time = conv_time(self.get_clock().now().to_msg())
        self.ready = True

        self.Hz = 2
        self.dt = 1 / self.Hz
        self._i = 0

        self.goal_handle = None
        self.target = None
        self.position = [0, 0]
        self.orientation = 0.

        self._load_parameters()

        self.get_logger().info('Navigator ready')

    def _load_parameters(self):
        default_params = [
            ('min_distance', 0.7),
            ('speed_factor', 1.),
            ('turn_factor', 2.3),
            ('steering_limit', math.pi / 8),
        ]
        self.declare_parameters(
            namespace='',
            parameters=default_params
        )

        load_param = lambda param: self.get_parameter(param).get_parameter_value().double_value

        self.navigation_params = {
            param: load_param(param) for param, _ in default_params
        }

    def execute_callback(self, goal_handle):
        # IDEA: check distance to target -> reject if too far away
        self.goal_handle = goal_handle
        self.target = goal_handle.request.target.x, goal_handle.request.target.y
        self.get_logger().info('Targeting position ({}, {})'.format(*self.target))

        while not self.stopping_criterion():
            if self.ready:
                self.ready = False
                self.do_navigation()
                self.send_feedback_information()
                self.log_distance()
                time.sleep(0.01)

        # -- DONE --
        self.send_stop_order()

        self.get_logger().info("Navigation to target complete")
        self.goal_handle.succeed()
        self.goal_handle = None
        self.target = None

        result = DriveToLocation.Result()
        result.final_position = self.get_pose()
        return result

    def do_navigation(self):
        angle = self.get_angle_to_target()

        if abs(angle) > self.navigation_params['steering_limit']:
            angle = math.copysign(self.navigation_params['steering_limit'], angle)

        self.send_movement_order(angle_to_vector(angle), speed=self.navigation_params['speed_factor'])

    def log_distance(self):
        self._i += 1
        if self._i > self.Hz:
            self.get_logger().info("Distance to target: %f" % distance_between(self.position, self.target))
            self._i = 0

    def send_feedback_information(self):
        feedback_msg = DriveToLocation.Feedback()
        feedback_msg.current_location = self.get_pose()
        self.goal_handle.publish_feedback(feedback_msg)

    def incoming_position_update(self, msg):
        self.position = [msg.x, msg.y]
        self.orientation = msg.theta

    def get_angle_to_target(self):
        if self.target is None:
            return 0.0

        theta = math.atan2(self.target[1] - self.position[1], self.target[0] - self.position[0])
        angle = theta - self.orientation

        if angle > math.pi:
            angle -= 2 * math.pi

        angle *= self.navigation_params['turn_factor']

        return angle

    def stopping_criterion(self):
        return distance_between(self.position, self.target) < self.navigation_params['min_distance'] if self.target is not None else True

    # -- utility functions --
    def get_pose(self):
        return toWaypoint(*self.position, self.orientation)

    # -- mov interface functions --
    def send_movement_order(self, movement, speed=1.0, normalize=True):
        mov = Mov()
        fwd, rot = movement
        mov.forward = fwd
        mov.rotate = rot

        if normalize:
            length = math.sqrt(fwd * fwd + rot * rot)
            if length > 0:
                mov.forward /= length
                mov.rotate /= length
            else:
                mov.forward = 0.0
                mov.rotate = 0.0

        mov.forward *= speed
        mov.rotate *= speed

        self.movement_order_publisher.publish(mov)

    def send_stop_order(self):
        msg = Mov()
        msg.forward = 0.0
        msg.rotate = 0.0
        self.movement_order_publisher.publish(msg)
        self.get_logger().info('Stopping motor')

    def check_time(self, current_time):
        diff = conv_time(current_time) - self.last_time
        if diff > self.dt:
            self.last_time += self.dt
            self.ready = True


def toWaypoint(x, y, theta):
    pos = Waypoint()
    pos.x = float(x)
    pos.y = float(y)
    pos.theta = float(theta)
    return pos

def angle_between(v1, v2):
    v1x, v1y = v1
    v2x, v2y = v2
    return math.atan2(v2y, v2x) - math.atan2(v1y, v1x)

def distance_between(x, y):
    x1, y1 = x
    x2, y2 = y
    return math.hypot(x2 - x1, y2 - y1)

def vetor_to_angle(v):
    return math.atan2(v[1], v[0])

def angle_to_vector(a):
    return [math.cos(a), math.sin(a)]

def conv_time(current_time):
    return current_time.sec + current_time.nanosec * 1e-9


def main(args=None):
    rclpy.init(args=args)

    try:
        navigator = Navigator()
        executor = MultiThreadedExecutor(num_threads=4)
        executor.add_node(navigator)

        try:
            executor.spin()
        finally:
            executor.shutdown()
            navigator.send_stop_order()
            navigator.destroy_node()

    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
