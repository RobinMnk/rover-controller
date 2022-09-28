import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer

from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup

from motor_interfaces.action import WaypointService
from motor_interfaces.msg import WaypointData

import time
import random

class WaypointServiceNode(Node):
    def __init__(self):
        self.group = ReentrantCallbackGroup()
        super().__init__('WaypointServiceNode')
        self.srv = ActionServer(
            self,
            WaypointService,
            'waypoint_service',
            self.new_task,
            callback_group=self.group
        )

    def new_task(self, goal_handle):
        wait_time = random.randint(3, 8)

        self.get_logger().info("Begin Service at Waypoint {}. Estimated time: {}s ... ".format(goal_handle.request.waypoint_index, wait_time))
        time.sleep(wait_time)
        self.get_logger().info("Finished work at Waypoint {} !".format(goal_handle.request.waypoint_index))

        data = WaypointData()
        data.waypoint_index = goal_handle.request.waypoint_index
        data.depth = random.uniform(14, 30)
        data.confidence = random.uniform(0, 1)
        data.time_spent = wait_time
        data.errors = ""

        goal_handle.succeed()

        response = WaypointService.Result()
        response.data = data
        return response

def main(args=None):
    rclpy.init(args=args)

    try:
        waypoint_service = WaypointServiceNode()
        executor = MultiThreadedExecutor(num_threads=2)
        executor.add_node(waypoint_service)

        try:
            executor.spin()
        finally:
            executor.shutdown()
            waypoint_service.destroy_node()

    finally:
        rclpy.shutdown()
if __name__ == '__main__':
    main()