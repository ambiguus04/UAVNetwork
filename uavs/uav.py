import math
import random


class Vector(object):

    def __init__(self, x=0, y=0, z=0):
        self.x = x
        self.y = y
        self.z = z

    def subtract(self, start):
        return Vector(self.x - start.x, self.y - start.y, self.z - start.z)

    def add(self, a):
        self.x += a.x
        self.y += a.y
        self.z += a.z

    def get_tuple(self):
        return self.x, self.y, self.z

    def length(self, end=None):
        if not end:
            end = Vector()
        return math.sqrt((end.x - self.x) ** 2 + (end.y - self.y) ** 2 + (end.z - self.z) ** 2)

    def length_norm(self, end, area_size):
        x = abs(end.x - self.x)
        move = abs(end.x - self.x + area_size)
        if move < x:
            x = move
        y = abs(end.y - self.y)
        move = abs(end.y - self.y + area_size)
        if move < y:
            y = move
        z = abs(end.z - self.z)
        move = abs(end.z - self.z + area_size)
        if move < z:
            z = move
        return math.sqrt(x*x + y*y + z*z)

    def print_elements(self):
        print("({:.2f}, {:.2f}, {:.2f})".format(self.x, self.y, self.z))

    def direction(self, pos, norm_length, area_size):
        x, y = pos.x - self.x, pos.y - self.y
        length = math.sqrt(x * x + y * y)
        norm = norm_length / length
        x *= norm
        y *= norm
        if self.length(pos) > self.length_norm(pos, area_size):
            return -x, -y
        return x, y

    def length_x_y(self):
        return math.sqrt(self.x * self.x + self.y * self.y)


class UAV(object):

    def __init__(self, x=0, y=0, z=0, vx=0, vy=0, vz=0):
        self.pos = Vector(x, y, z)
        self.v = Vector(vx, vy, vz)
        self.g = -10
        self.random_fly = True
        self.v_xy_norm = 1
        self.known_thermals = []
        self.area_size = 1000

    def fly(self, time):
        step = Vector(self.v.x * time, self.v.y * time, self.v.z * time)
        self.pos.add(step)
        # self.pos.print_elements()

    def height(self):
        return self.pos.z

    def go_to(self, pos):
        length = self.v.length_x_y()
        vx, vy = self.pos.direction(pos, length, self.area_size)
        self.v.x = vx
        self.v.y = vy

    def set_velocity(self, v):
        self.v.z = v.z
        norm = v.x*v.x + v.y*v.y
        self.v.x = v.x * self.v_xy_norm / norm
        self.v.y = v.y * self.v_xy_norm / norm

    def set_velocity_xy(self, x, y):
        norm = x*x + y*y
        self.v.x = x * self.v_xy_norm / norm
        self.v.y = y * self.v_xy_norm / norm

    def gaussian_direction(self, alpha=0, sigma=45):
        mu = math.radians(alpha)
        sigma = math.radians(sigma)
        rad = random.gauss(mu, sigma)
        if rad < 0:
            rad += 2 * math.pi
        if rad > 2 * math.pi:
            rad -= 2 * math.pi
        r = self.v_xy_norm
        self.v.x = r * math.cos(rad)
        self.v.y = r * math.sin(rad)

    def add_thermal(self, thermal):
        self.known_thermals.append(thermal)

    @property
    def nearest_thermal(self):
        if not self.known_thermals:
            return None
        nearest = None
        nearest_dist = 100000
        for thermal in self.known_thermals:
            d = thermal.length(self.pos)
            if d < nearest_dist:
                nearest = thermal
                nearest_dist = d
        return nearest

    def go_to_nearest_thermal(self):
        self.go_to(self.nearest_thermal)

    def is_known(self, thermal):
        for known in self.known_thermals:
            if known.same(thermal):
                return True
        return False

