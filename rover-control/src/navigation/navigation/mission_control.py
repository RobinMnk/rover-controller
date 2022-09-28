import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from motor_interfaces.action import MissionAction
from motor_interfaces.msg import Waypoint

import os
import threading
from shutil import move as move_file
import time
from datetime import date, datetime
from collections import namedtuple
import utm

from .utility.JSON_parser import read_JSON, save_JSON

MISSION_LOG_DIRECTORY_NAME = "missions"
INCOMING_MISSION_FILENAME = "mission.json"

Mission = namedtuple("Mission", "id route")

""" 
    MissionControl
    responsible for initiating the route and logging the results 
"""
class MissionControl(Node):
    def __init__(self):
        super().__init__('MissionControl')
        self.trip_planner_client = ActionClient(
            self,
            MissionAction,
            'mission',
        )

        # global file setup for all missions
        self.server_home = None
        self.incoming_mission_file = None
        self.mission_log_directory = None

        # files related to one mission
        self.mission_home = None
        self.current_mission_file = None
        self.current_mission = None

        self.setup()

    def setup(self):
        self.server_home = os.environ.get('SERVER_HOME', None)

        if self.server_home is None:
            raise RuntimeError("The environment variable SERVER_HOME is not set!")

        self.incoming_mission_file = os.path.join(self.server_home, INCOMING_MISSION_FILENAME)
        self.mission_log_directory = os.path.join(self.server_home, MISSION_LOG_DIRECTORY_NAME)
        os.makedirs(self.mission_log_directory, exist_ok=True)

        thread = threading.Thread(target=self.main_loop, daemon=True)
        thread.start()

        self.get_logger().info('Mission Control ready')

    def main_loop(self):
        while rclpy.ok():
            if self.current_mission is None and os.path.isfile(self.incoming_mission_file):
                self.begin_new_mission(self.incoming_mission_file)

            time.sleep(2)

    def begin_new_mission(self, new_mission_file):
        self.get_logger().info("\n" + "-" * 32 + "\n" + "-" * 7 + " STARTING MISSION " + "-" * 7 + "\n" + "-" * 32)
        mission_data = read_JSON(new_mission_file)
        self.current_mission = Mission(mission_data['mission_id'], mission_data['route'])

        self.mission_home = os.path.join(self.mission_log_directory, "mission_" + self.current_mission.id)
        os.makedirs(self.mission_home, exist_ok=True)
        self.current_mission_file = os.path.join(self.mission_home, "mission.json")

        move_file(self.incoming_mission_file, self.current_mission_file)

        mission_data['started_at'] = create_current_timestamp()
        save_JSON(self.current_mission_file, mission_data)

        self.send_goal(self.current_mission)

    def finish_current_mission(self):
        mission_data = read_JSON(self.current_mission_file)
        mission_data['finished_at'] = create_current_timestamp()
        save_JSON(self.current_mission_file, mission_data)

        self.mission_home = None
        self.current_mission_file = None
        self.current_mission = None

    def send_goal(self, mission: Mission):
        self.get_logger().info('Sending goal!')
        goal_msg = MissionAction.Goal()
        waypoints = list(map(convert_to_waypoint, mission.route))
        goal_msg.targets = waypoints

        self.trip_planner_client.wait_for_server()

        goal_future = self.trip_planner_client.send_goal_async(
            goal_msg,
            feedback_callback=self.waypoint_reached
        )

        goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()

        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        self.get_logger().info("\n" + "-" * 32 + "\n" + "-" * 7 + " MISSION FINISHED " + "-" * 7 + "\n" + "-" * 32)
        result = future.result().result
        self.finish_current_mission()

    def waypoint_reached(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info('Navigated to target {0} in {1}s'.format(feedback.last_completed, feedback.time_for_navigation))
        waypoint_data_msg = feedback.last_waypoint_data

        if waypoint_data_msg is not None:
            waypoint_data = {
                'waypoint_index': feedback.last_completed + 1,
                'navigation_time': feedback.time_for_navigation,
                'depth': waypoint_data_msg.depth,
                'confidence': waypoint_data_msg.confidence,
                'time_spent': waypoint_data_msg.time_spent,
                'errors': waypoint_data_msg.errors
            }

            directory_name = 'waypoint_' + str(waypoint_data['waypoint_index'])
            directory_path = os.path.join(self.mission_home, directory_name)
            os.makedirs(directory_path,  exist_ok=True)
            waypoint_file = os.path.join(directory_path, "waypoint_data.json")
            save_JSON(waypoint_file, waypoint_data)

def convert_to_waypoint(entry):
    u = utm.from_latlon(entry['latitude'], entry['longitude'])
    wpt = Waypoint()
    wpt.x = float(u[0])
    wpt.y = float(u[1])
    wpt.theta = 0.0
    wpt.skip_work = entry.get("skip_work", False)
    return wpt

def create_current_timestamp():
    today = date.today()
    now = datetime.now()
    return "%s:%s" % (today.strftime("%Y-%m-%d"), now.strftime("%H-%M-%S"))

def main(args=None):
    rclpy.init(args=args)

    mission_control = MissionControl()

    rclpy.spin(mission_control)

    mission_control.destroy_node()
    rclpy.shutdown()


def toWaypoint(x, y):
    pos = Waypoint()
    pos.x = float(x)
    pos.y = float(y)
    pos.theta = 0.0
    return pos

if __name__ == '__main__':
    main()
