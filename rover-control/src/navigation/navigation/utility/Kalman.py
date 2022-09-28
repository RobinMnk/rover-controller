from filterpy.kalman import KalmanFilter
from filterpy.common import Q_discrete_white_noise

import numpy as np

class KalmanLib(object):
    def __init__(self, kalman_params=None):
        self.filter = KalmanFilter(dim_x=4, dim_z=2)
        self.ready = False
        self.params = kalman_params or {}

        self.x = np.matrix('0. 0. 0. 0.').T
        self.P = self.params.get('kalman_P', 1000.) * np.eye(4)
        self.R = self.params.get('kalman_R', 0.0001)
        self.Q = Q_discrete_white_noise(
            dim=4,
            dt=self.params.get('kalman_Q_dt', .1),
            var=self.params.get('kalman_Q_var', .13)
        )
        self.F = np.matrix('''
                          1. 0. 1. 0.;
                          0. 1. 0. 1.;
                          0. 0. 1. 0.;
                          0. 0. 0. 1.
                          ''')
        self.H = np.matrix('''
                          1. 0. 0. 0.;
                          0. 1. 0. 0.'''
                           )

    def setup(self, start_x=0., start_y=0.):
        self.filter.x = np.transpose(np.array([[start_x, start_y, 0., 0]]))

        self.filter.F = np.array([ [1., 0., 1., 0.],
                                   [0., 1., 0., 1.],
                                   [0., 0., 1., 0.],
                                   [0., 0., 0., 1.] ])

        self.filter.H = np.array([ [1., 0., 0., 0.],
                                    [0., 1., 0., 0.] ])

        self.filter.P *= self.P
        self.filter.R = self.R
        self.filter.Q = self.Q

        self.ready = True

    def apply2(self, new_position):
        if not self.ready:
            return [0., 0.]

        z = np.transpose(np.array([ list(new_position) ]))
        self.filter.predict()
        self.filter.update(z)
        return np.transpose(self.filter.x)[0][:2]

class KalmanManually(object):
    def __init__(self, kalman_params=None):
        self.ready = False
        self.params = kalman_params or {}

    def setup(self, start_x=0., start_y=0.):
        self._setup_parameters(start_x, start_y)

    def _setup_parameters(self, start_x, start_y):
        self.x = np.matrix('{} {} 0. 0.'.format(start_x, start_y)).T
        self.P = self.params.get('kalman_P', 1000.) * np.eye(4)
        self.R = self.params.get('kalman_R', 0.0001)
        self.Q = Q_discrete_white_noise(
            dim=4,
            dt=self.params.get('kalman_Q_dt', .1),
            var=self.params.get('kalman_Q_var', .13)
        )
        self.F = np.matrix('''
                          1. 0. 1. 0.;
                          0. 1. 0. 1.;
                          0. 0. 1. 0.;
                          0. 0. 0. 1.
                          ''')
        self.H = np.matrix('''
                          1. 0. 0. 0.;
                          0. 1. 0. 0.'''
                          )
        self.ready = True

    def __kalman(self, measurement):
        # UPDATE x, P based on measurement m
        # distance between measured and current position-belief
        y = np.matrix(measurement).T - self.H * self.x
        S = self.H * self.P * self.H.T + self.R  # residual convariance
        K = self.P * self.H.T * S.I  # Kalman gain
        x = self.x + K * y
        I = np.matrix(np.eye(4))  # identity matrix
        P = (I - K * self.H) * self.P

        x = self.F * x
        P = self.F * P * self.F.T + self.Q

        return x, P

    def apply(self, new_position):
        if not self.ready:
            return [0., 0.]

        res = self.__kalman(new_position)
        self.x = res[0]
        self.P =res[1]

        return list(map(lambda x: x.tolist()[0][0], self.x[:2]))
