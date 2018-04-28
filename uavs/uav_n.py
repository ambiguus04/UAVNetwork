import numpy as np
import math

__author__ = 'marta'


def is_empty(a):
    return a.size == 0


class Environment(object):
    radius = 1
    close = 0.5

    def __init__(self, n=1, area_size=1000):
        self.area_size = area_size
        self.area = np.array((area_size, area_size))
        self.thermals = np.multiply(np.random.random((n, 2)), self.area_size)

    def per_subtract(self, x, y):
        return np.mod(np.subtract(x, y), self.area)

    def vec_to_thermals(self, uav, thermals=None):
        uav_xy = uav[:2].reshape((1, 2))
        if thermals is None:
            thermals = self.thermals
        y1 = self.per_subtract(uav_xy, thermals)
        y2 = self.per_subtract(thermals, uav_xy)
        np.putmask(y1, y1 > y2, y2)
        return y1

    def dist_to_thermals(self, uav, thermals=None):
        return np.linalg.norm(self.vec_to_thermals(uav, thermals), axis=1)

    def nearest_thermal(self, uav, thermals=None):
        if thermals is None:
            thermals = self.thermals
        return thermals[np.argmin(self.dist_to_thermals(uav, thermals))]

    def in_distance(self, uav, dist):
        d = self.dist_to_thermals(uav)
        in_dist = np.where(np.less(d, dist))
        return self.thermals[in_dist]  # positions of thermals in distance dist

    def in_thermal(self, uav):
        return self.in_distance(uav, self.radius)

    def in_middle(self, uav):
        return self.in_distance(uav, self.close)

    def normalize_pos(self, uav):
        return np.mod(uav, self.area)


class UAV(object):
    def __init__(self, height=10, area_size=1000, v_falling=-0.1, v_norm=1, forget_rate=0):
        self.pos = np.zeros(3)
        self.pos[:2] += np.multiply(np.random.random(2), area_size)
        self.pos[2] = height
        self.area_size = area_size
        self.area = np.array([area_size, area_size])
        self.v = np.zeros(3)
        self.v_norm = v_norm
        self.v[0:2] = self.gaussian_direction()
        self.v_falling = v_falling
        self.v[2] = self.v_falling
        self.known_thermals = []
        self.random_flight = True
        self.forget_rate = forget_rate
        self.time_thermals = []

    def gaussian_direction(self, mu=0, sigma=45):
        mu = math.radians(mu)
        sigma = math.radians(sigma)
        alpha = np.mod(np.random.normal(mu, sigma), 2*np.pi)
        r = self.v_norm
        self.v[0] = np.multiply(np.cos(alpha), r)
        self.v[1] = np.multiply(np.sin(alpha), r)
        return np.hstack((np.multiply(np.cos(alpha), r), np.multiply(np.sin(alpha), r)))

    @property
    def height(self):
        return self.pos[2]

    def set_v_z(self, v_z):
        self.v[2] = v_z

    def move_to_thermal(self):
        scale = 20  # start somewhere in radius 10 from thermal
        if len(self.known_thermals) > 0:
            thermal = self.known_thermals[0]
            add = np.subtract(np.multiply(np.random.random(2), scale), scale * 0.5)
            self.pos[0] = thermal[0] + add[0]
            self.pos[1] = thermal[1] + add[1]

    def is_known_thermal(self, thermal):
        t = list(thermal)[0]
        return any(np.all(np.equal(t, known)) for known in self.known_thermals)

    def add_thermal(self, thermal):
        for t in list(thermal):
            self.known_thermals.append(t)
            if self.time_thermals is not None:
                self.time_thermals.append(self.forget_rate)

    def rise(self, v_rising):
        self.v[:2] = 0
        self.v[2] = v_rising

    def norm_pos(self, x, y):
        return np.vstack((np.mod(np.subtract(x, y), self.area), (np.mod(np.subtract(y, x), self.area))))

    def min_length(self, x, y):
        np.min(self.norm_pos(x, y), axis=1)

    def go_to(self, pos):
        x_d = self.pos[0]
        y_d = self.pos[1]
        x_t = pos[0]
        y_t = pos[1]
        a = self.area_size
        x_diff = x_t - x_d
        x = x_diff  # default
        if abs(x_diff) > abs(a - abs(x_diff)):  # around
            if x_t > x_d:
                x = x_diff - a
            else:
                x = a + x_diff

        y_diff = y_t - y_d
        y = y_diff
        if abs(y_diff) > abs(a - abs(y_diff)):
            if y_t > y_d:
                y = y_diff - a
            else:
                y = a + y_diff

        length = np.linalg.norm(np.array([x, y]))
        norm = self.v_norm / length
        self.v[0] = x * norm
        self.v[1] = y * norm

    def fly(self, time):
        step = np.multiply(self.v, time)
        np.add(self.pos, step, self.pos)

    def normalize_pos(self):
        self.pos[0:2] = np.mod(self.pos[0:2], self.area)

    def count_thermals_time(self, time):
        if self.time_thermals is not None:
            for i, t in enumerate(self.time_thermals):
                self.time_thermals[i] -= time
                if t <= 0:
                    self.known_thermals.pop(i)
                    self.time_thermals.pop(i)  # forgeting

    def update_thermal_time(self, thermal):
        t = list(thermal)[0]
        index = None
        for i, known in enumerate(self.known_thermals):
            if np.all(np.equal(t, known)):
                index = i
        if index is not None:
            self.time_thermals[index] = self.forget_rate


class Swarm(object):
    v_falling = -0.1
    v_rising = 0.2
    v_norm = 1  # norm of velocity xy

    def __init__(self, size=1, height=10, sigma=45, area_size=1000, thermals=1000, verbose=False, forget_rate=0):
        self.area_size = area_size
        self.size = size
        self.height = height
        self.min_height = 0.5 * height
        self.sigma = sigma
        self.forget_rate = forget_rate
        self.uavs = [UAV(self.height, self.area_size, self.v_falling, self.v_norm, forget_rate=self.forget_rate)]
        self.env = Environment(thermals, area_size)
        self.thermals = thermals
        self.set_nearest_thermals()
        self.verbose = verbose

    def set_nearest_thermals(self):
        for uav in self.uavs:
            uav.known_thermals.append(self.env.nearest_thermal(uav.pos))
            # if uav.time_thermals is not None:
            uav.time_thermals.append(self.forget_rate)
            uav.move_to_thermal()
            uav.normalize_pos()

    def nearest_thermal(self, uav):
        pos = uav.pos
        if len(uav.known_thermals) <= 0:
            return None
        thermals = np.array(uav.known_thermals)
        # print(thermals)
        return self.env.nearest_thermal(pos, thermals)

    def fly(self, steps=200000):
        step_length = 0.5
        alpha = 0
        v_cum = 0
        delay = 0.1
        for ii in range(0, steps):
            for uav in self.uavs:
                uav.random_flight = True
                thermal = self.env.in_thermal(uav.pos)
                if not is_empty(thermal):
                    if self.verbose:
                        print("in thermal")
                    uav.set_v_z(self.v_rising)
                    if not uav.is_known_thermal(thermal):
                        if self.verbose:
                            print("thermal i know")
                            print(thermal)
                            print(uav.known_thermals)
                        uav.add_thermal(thermal)
                        if self.verbose:
                            print("adding")
                            print(uav.known_thermals)
                    else:
                        uav.update_thermal_time(thermal)
                    if uav.height < self.height and not is_empty(self.env.in_middle(uav.pos)):
                        uav.random_flight = False
                        if self.verbose:
                            print("stop")
                        if uav.height < self.min_height:
                            uav.rise(self.v_rising)
                            if self.verbose:
                                print("up")
                    if uav.height >= self.height:
                        uav.random_flight = True
                else:
                    uav.set_v_z(self.v_falling)
                    if uav.height < self.min_height:
                        if self.verbose:
                            print("look for thermal")
                        nearest = self.nearest_thermal(uav)
                        if nearest is not None:
                            uav.go_to(nearest)
                            uav.random_flight = False
                if ii % 10 and self.verbose:
                    print(uav.v[0])
                    print(uav.pos)
                if ii % 10 == 0 and uav.random_flight:
                    if self.verbose:
                        print("random flight")
                    uav.gaussian_direction(alpha, self.sigma)
                uav.fly(step_length)
                uav.normalize_pos()
                if ii >= steps * delay:
                    v_cum += uav.v[0]
                uav.count_thermals_time(step_length)
                if uav.height < -2:
                    v_cum = 0
                    print(str(self.thermals) + "\t" + str(v_cum))
                    return
        v_cum /= (steps * step_length * (1 - delay))
        print(str(self.thermals) + "\t" + str(v_cum))
