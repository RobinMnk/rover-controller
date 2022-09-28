import os
from abc import ABC, abstractmethod

import utm
from datetime import date, datetime

from .JSON_parser import read_JSON, save_JSON

ROVER_DATA_FILENAME = "rover_data.json"

class LoggingSystem(object):
    LOG_POSITION_TYPE_INTERMEDIATE = 0
    LOG_POSITION_TYPE_FINAL = 1

    def __init__(self):
        self.loggers = []

    def setup(self, mission_id, route):
        for logger in self.loggers:
            logger.setup(mission_id, route)

    def log_location(self, location, utm_position, logtype=LOG_POSITION_TYPE_INTERMEDIATE):
        for logger in self.loggers:
            logger.log_location(location, utm_position, logtype)

    def register_logger(self, logger):
        self.loggers.append(logger)

    def end_route(self):
        for logger in self.loggers:
            logger.end_route()

class _RouteLogger(ABC):

    def __init__(self, frequency=1):
        self.logging_root = None
        self.logging_home = None
        self.logging_filename = None

        self.mission_id = ""

        self.frequency = frequency
        self.i = 0

    @abstractmethod
    def setup(self, mission_id, route): pass

    def log_location(self, location, utm_position, logtype=LoggingSystem.LOG_POSITION_TYPE_INTERMEDIATE):
        if logtype == LoggingSystem.LOG_POSITION_TYPE_FINAL:
            self.log_final_location(location, utm_position)
        elif logtype == LoggingSystem.LOG_POSITION_TYPE_INTERMEDIATE:
            self.i += 1
            if self.i >= self.frequency:
                self.i = 0
                self.log_intermediate_location(location, utm_position)

    @abstractmethod
    def log_intermediate_location(self, location, utm_position): pass

    @abstractmethod
    def log_final_location(self, location, utm_position): pass

    @abstractmethod
    def end_route(self): pass


class DefaultRouteLogger(_RouteLogger):
    def setup(self, mission_id, route):
        self.logging_root = os.environ.get('ROUTE_LOGGING_HOME', None)

        if self.logging_root is None:
            raise RuntimeError("The environment variable ROUTE_LOGGING_HOME is not set!")

        self.mission_id = mission_id

        today = date.today()
        directory_name = today.strftime("%Y-%m-%d")
        date_directory = os.path.join(self.logging_root, directory_name)

        if not os.path.exists(date_directory):
            os.makedirs(date_directory)

        now = datetime.now()

        current_time = now.strftime("%H-%M-%S")

        self.logging_filename = "Run_" + current_time + ".log"
        self.log_dir_and_file = os.path.join(directory_name, self.logging_filename)

        self.logging_home = os.path.join(date_directory, self.logging_filename)

        with open(self.logging_home, 'a') as f:
            f.write("Route with {} stops:\n".format(len(route)))

            for location in route:
                f.write("{} {}\n".format(location.x, location.y))

            f.write("- - -\n")

    def log_final_location(self, location, _):
        with open(self.logging_home, 'a') as f:
            f.write("f {} {} {}\n".format(location.x, location.y, location.theta))
            f.write("- - -\n")

    def log_intermediate_location(self, location, _):
        with open(self.logging_home, 'a') as f:
            f.write("i {} {} {}\n".format(location.x, location.y, location.theta))

    def end_route(self):
        with open(self.logging_home, 'a') as f:
            f.write("- - -\nEND\n")

        os.system(" ".join([
            "python3",
            os.path.join(os.getcwd(), "utils/route_visualizer.py"),
            self.log_dir_and_file
        ]))


class RouteLoggerJSON(_RouteLogger):
    def setup(self, mission_id, route):
        self.logging_root = os.environ.get('SERVER_HOME', None)

        if self.logging_root is None:
            raise RuntimeError("The environment variable SERVER_HOME is not set!")

        self.mission_id = mission_id

        self.logging_filename = ROVER_DATA_FILENAME
        self.logging_home = os.path.join(self.logging_root, self.logging_filename)

        data = self.load()
        data['mission_id'] = mission_id
        data['last_waypoint'] = 0
        self.save(data)

    def log_final_location(self, _, utm_position):
        if utm_position is None:
            return
        lat, lon = utm.to_latlon(utm_position.x, utm_position.y, utm_position.zone_nr, utm_position.zone_str)
        data = self.load()
        data['latitude'] = lat
        data['longitude'] = lon
        data['last_waypoint'] += 1
        self.save(data)

    def log_intermediate_location(self, _, utm_position):
        if utm_position is None:
            return
        lat, lon = utm.to_latlon(utm_position.x, utm_position.y, utm_position.zone_nr, utm_position.zone_str)
        data = self.load()
        data['latitude'] = lat
        data['longitude'] = lon
        self.save(data)

    def end_route(self):
        data = self.load()
        data['mission_id'] = ""
        data['last_waypoint'] = -1
        self.save(data)

    def save(self, jsonObject):
        save_JSON(self.logging_home, jsonObject)

    def load(self):
        return read_JSON(self.logging_home)
