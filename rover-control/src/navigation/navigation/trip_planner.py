import rclpy
from rclpy.action import ActionServer, ActionClient
from rclpy.node import Node

from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup

from motor_interfaces.msg import UTM
from motor_interfaces.action import WaypointService
from motor_interfaces.action import MissionAction, DriveToLocation

from .utility.route_logger import DefaultRouteLogger, LoggingSystem, RouteLoggerJSON

import time

class TripPlanner(Node):
    def __init__(self):
        super().__init__('TripPlanner')
        self.group = ReentrantCallbackGroup()
        self.mission_action_server = ActionServer(
            self,
            MissionAction,
            'mission',
            self.execute_callback,
            callback_group=self.group
        )

        self.navigator_client = ActionClient(
            self,
            DriveToLocation,
            'drive_to_location',
            callback_group=self.group
        )

        self.waypoint_service_client = ActionClient(
            self,
            WaypointService,
            'waypoint_service',
            callback_group=self.group
        )

        self.utm_subscriber = self.create_subscription(UTM, 'utm', self._incoming_utm_signal, 10, callback_group=self.group)
        self.last_utm_signal = None

        self.temp_result = None

        self.goal_handle = None
        self.current_route = None
        self.current_target_index = None
        self.mission_id = ""
        self.navigation_start_time = 0

        self.route_logger = LoggingSystem()
        self.route_logger.register_logger(DefaultRouteLogger())
        # self.route_logger.register_logger(RouteLoggerJSON(2))

        self.get_logger().info('Trip Planner ready')

    def execute_callback(self, goal_handle):
        self.get_logger().info('Received new tour with {} stops'.format(len(goal_handle.request.targets)))

        while self.last_utm_signal is None:
            self.get_logger().info('Waiting for Localization Service ...')
            time.sleep(2)

        self.goal_handle = goal_handle
        self.current_route = goal_handle.request.targets
        self.current_target_index = 0
        self.mission_id = goal_handle.request.mission_id

        trip_start_time = time.time()

        self.route_logger.setup(self.mission_id, self.current_route)

        result_positions = []

        while self.current_target_index < len(self.current_route):
            self.get_logger().info('Targeting new location, now at {}/{}'.format(self.current_target_index, len(self.current_route)))

            """ Navigate to Waypoint """
            self.temp_result = None
            self.send_navigation_order()

            while self.temp_result is None:
                time.sleep(1)

            result_positions.append(self.temp_result)


            """ Execute Work at Waypoint """
            if not self.current_route[self.current_target_index].skip_work:
                self.get_logger().info('Executing work at Waypoint')

                self.temp_result = None
                self.send_waypoint_work_order()

                while self.temp_result is None:
                    time.sleep(1)

            else:
                self.get_logger().info('Skipping work at Waypoint')

            self.current_target_index += 1

        # -- DONE --

        goal_handle.succeed()
        self.finish_route()

        msg = MissionAction.Result()
        msg.final_positions = result_positions
        msg.total_time = int(time.time() - trip_start_time)
        return msg


    def send_waypoint_work_order(self):
        goal_msg = WaypointService.Goal()
        goal_msg.waypoint_index = self.current_target_index

        self.waypoint_service_client.wait_for_server()

        navigator_handle = self.waypoint_service_client.send_goal_async(goal_msg)
        navigator_handle.add_done_callback(self.waypoint_service_response_callback)

    def waypoint_service_response_callback(self, future):
        goal_handle = future.result()

        if goal_handle.accepted:
            result_future = goal_handle.get_result_async()
            result_future.add_done_callback(self.send_waypoint_service_feedback)
        else:
            self.get_logger().info('Goal rejected :(')

    def send_waypoint_service_feedback(self, service_feedback):
        """ Send feedback to MissionControl about the result of WaypointService """
        self.get_logger().info("Returned from Waypoint Service")
        self.temp_result = service_feedback.result().result.data

        feedback_msg = MissionAction.Feedback()
        feedback_msg.last_completed = self.current_target_index
        feedback_msg.time_for_navigation = int(time.time() - self.navigation_start_time)
        feedback_msg.last_waypoint_data = self.temp_result
        self.goal_handle.publish_feedback(feedback_msg)

    def send_navigation_order(self):
        goal_msg = DriveToLocation.Goal()
        goal_msg.target = self.current_route[self.current_target_index]

        self.navigation_start_time = time.time()
        self.navigator_client.wait_for_server()

        navigator_handle = self.navigator_client.send_goal_async(
            goal_msg, feedback_callback=self.navigator_feedback_callback
        )
        navigator_handle.add_done_callback(self.navigator_response_callback)

    def navigator_response_callback(self, future):
        goal_handle = future.result()

        if goal_handle.accepted:
            result_future = goal_handle.get_result_async()
            result_future.add_done_callback(self.navigator_reached_target)
        else:
            self.get_logger().info('Goal rejected :(')

    def navigator_reached_target(self, future):
        """ Send feedback to MissionControl about having navigated to the next target """
        self.temp_result = future.result().result.final_position
        self.get_logger().info('Navigator has reached target {0}'.format(self.current_target_index))
        self.get_logger().info('\t -> final position: {0}, {1}'.format(self.temp_result.x, self.temp_result.y))

        feedback_msg = MissionAction.Feedback()
        feedback_msg.last_completed = self.current_target_index
        feedback_msg.time_for_navigation = int(time.time() - self.navigation_start_time)
        self.goal_handle.publish_feedback(feedback_msg)

        self.route_logger.log_location(self.temp_result, self.last_utm_signal, logtype=LoggingSystem.LOG_POSITION_TYPE_FINAL)

    def navigator_feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback.current_location
        self.route_logger.log_location(feedback, self.last_utm_signal)

    def finish_route(self):
        self.get_logger().info('The route has been finished!')

        self.goal_handle = None
        self.current_route = None
        self.current_target_index = None
        self.route_logger.end_route()

    def _incoming_utm_signal(self, msg):
        self.last_utm_signal = msg


def main(args=None):
    rclpy.init(args=args)

    try:
        tripPlanner = TripPlanner()
        executor = MultiThreadedExecutor(num_threads=6)
        executor.add_node(tripPlanner)

        try:
            executor.spin()
        finally:
            tripPlanner.finish_route()
            executor.shutdown()
            tripPlanner.destroy_node()

    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
