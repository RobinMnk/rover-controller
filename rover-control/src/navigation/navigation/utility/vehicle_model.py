import math
import numpy as np

EARTH_RADIUS = 6378137

class VehicleModel(object):
    def __init__(self, dt):
        self.dt = dt

        self.speed_factor = 0.5 # meters per second
        self.turning_radius_factor = 100

    def _soften_angle(self, angle):
        return angle / self.turning_radius_factor

    def simulate(self, position, orientation, mov):
        speed = np.linalg.norm(mov)

        angle = vetor_to_angle(mov)

        new_orientation = angle_to_vector(orientation - self._soften_angle(angle))

        new_position = vector_add(position, list(map(
            lambda x: self.dt * speed * self.speed_factor * x,
        new_orientation)))

        # new_position = offset_lat_lon(position, list(map(
        #     lambda x: self.dt * speed * self.speed_factor * x,
        # new_orientation)))

        return new_position, vetor_to_angle(new_orientation)


def offset_lat_lon(position, offset):
    lat, lon = position
    off_lat, off_lon = offset
    dLat = off_lat / EARTH_RADIUS
    dLon = off_lon / (EARTH_RADIUS * math.cos(math.pi * lat / 180))
    return lat + dLat * 180 / math.pi, lon + dLon * 180 / math.pi


def vetor_to_angle(v): return math.atan2(v[1], v[0])

def angle_to_vector(a): return np.array([math.cos(a), math.sin(a)])

def vector_add(v1, v2):
    return [v1[i] + v2[i] for i in range(len(v1))]

def rotate(vector, angle):
    x, y = vector
    return  x * math.cos(angle) - y * math.sin(angle), \
            x * math.sin(angle) + y * math.cos(angle)