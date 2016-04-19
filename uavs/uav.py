__author__ = 'marta'

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
        return math.sqrt((end.x - self.x)**2 + (end.y - self.y)**2 + (end.z - self.z)**2)

    def print_elements(self):
        print("({:.2f}, {:.2f}, {:.2f})".format(self.x, self.y, self.z))

    def direction(self, pos, norm_length):
        x, y = pos.x - self.x, pos.y - self.y
        length = math.sqrt(x*x + y*y)
        norm = norm_length/length
        x *= norm
        y *= norm
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

    def fly(self,  time):
        step = Vector(self.v.x * time, self.v.y * time, self.v.z * time)
        self.pos.add(step)
        # self.pos.print_elements()

    def height(self):
        return self.pos.z

    def go_to(self, pos):
        length = self.v.length_x_y()
        vx, vy = self.pos.direction(pos, length)
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

    def gaussian_direction(self, alpha):
        mu = math.radians(alpha)
        sigma = math.radians(90)
        rad = random.gauss(mu, sigma)
        if rad < 0:
            rad += 2 * math.pi
        if rad > 2 * math.pi:
            rad -= 2 * math.pi
        r = self.v_xy_norm
        self.v.x = r * math.cos(rad)
        self.v.y = r * math.sin(rad)

