import math

class LinearRegression(object):
    def __init__(self, max_history=None):
        self.a = 1.0
        self.b = 0.0

        self.x = []
        self.y = []
        self.n = 0

        self._sum_xy = 0
        self._sum_x = 0
        self._sum_y = 0
        self._sum_x_sq = 0

        self.max_history = max_history

    def add_point(self, x_i, y_i):
        if self.max_history is not None and len(self.x) > self.max_history - 1:
            self.reset(self.max_history)

        self.x.append(x_i)
        self.y.append(y_i)
        self.n += 1

        self._sum_xy += x_i * y_i
        self._sum_x += x_i
        self._sum_y += y_i
        self._sum_x_sq += x_i * x_i

        if self.n > 1:
            self.__recalculate()

    def reset(self, keep_history=2):
        # IDEA: performance improvement: keep lower pointer and increase if necessary
        old_x = self.x[-keep_history:]
        old_y = self.y[-keep_history:]

        self.x = []
        self.y = []
        self.n = 0

        self._sum_xy = 0
        self._sum_x = 0
        self._sum_y = 0
        self._sum_x_sq = 0

        for p in zip(old_x, old_y):
            self.add_point(*p)

    def __recalculate(self):
        denom = self.n * self._sum_x_sq - self._sum_x * self._sum_x

        if denom != 0:
            self.a = (self.n * self._sum_xy - self._sum_x * self._sum_y) / denom
            self.b = (self._sum_y - self.a * self._sum_x) / self.n

    def angle(self):
        return math.atan(self.a)

    def history(self):
        return self.x, self.y

    def __str__(self):
        return "{} * x + {}\n\ttheta: {}".format(self.a, self.b, self.angle())


if __name__ == '__main__':
    regr = LinearRegression(2)
    regr.add_point(25, 0)
    regr.add_point(50, 14)
    print(regr)
    regr.add_point(60, 17)
    print(regr)
    regr.add_point(66, 17)
    print(regr)
    regr.add_point(77, -1)
    print(regr)
    regr.add_point(88, 0)
    print(regr)
    regr.add_point(100, -30)
    print(regr)
