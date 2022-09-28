from filterpy.kalman import KalmanFilter
from filterpy.common import Q_discrete_white_noise

import numpy as np


class Kalman(object):
    def __init__(self, start_x=0., start_y=0.):
        self.filter = KalmanFilter(dim_x=4, dim_z=2)
        self.filter.x = np.transpose(np.array([[start_x, start_y, 0., 0]]))

        self.filter.F = np.array([ [1., 0., 1., 0.],
                                   [0., 1., 0., 1.],
                                   [0., 0., 1., 0.],
                                   [0., 0., 0., 1.] ])

        self.filter.H = np.array([ [1., 0., 1., 0.],
                                    [0., 1., 0., 1.] ])

        self.filter.P *= 1000.

        self.filter.R = 5

        self.filter.Q = Q_discrete_white_noise(dim=4, dt=0.01, var=0.13)

    def apply(self, new_position):
        z = np.transpose(np.array([ list(new_position) ]))
        self.filter.predict()
        self.filter.update(z)
        return np.transpose(self.filter.x)[0]


f = Kalman(20, 7)

x = f.apply((0, 0))
for i in range(30):
    print(x)
    x = f.apply((0.1, 0))