from . import uav
import math
import random


class Thermal(uav.Vector):
    def __init__(self, x=0, y=0, z=0, area_size=1000):
        uav.Vector.__init__(self)
        self.x = x
        self.y = y
        self.z = z
        self.area_size = area_size

    def length(self, end=None):
        if not end:
            end = Thermal()
        x = abs(end.x - self.x)
        move = abs(end.x - self.x + self.area_size)
        if move < x:
            x = move
        y = abs(end.y - self.y)
        move = abs(end.y - self.y + self.area_size)
        if move < y:
            y = move
        return math.sqrt(x**2 + y**2)

    def same(self, thermal):
        if thermal.x != self.x:
            return False
        if thermal.y != self.y:
            return False
        return True


class Environment(object):
    def __init__(self, size=1, area_size=1000):
        self.area_size = area_size
        self.thermals = [Thermal(self.area_size * random.random(), self.area_size * random.random(), 0, self.area_size)
                         for _ in range(size)]
        self.radius = 1
        self.close = 0.5

    def in_distance(self, pos, dist):
        for thermal in self.thermals:
            if thermal.length(pos) <= dist:
                return True
        return False

    def in_thermal(self, pos):
        return self.in_distance(pos, self.radius)

    def in_middle(self, pos):
        return self.in_distance(pos, self.close)

    def get_thermal(self, pos):
        for thermal in self.thermals:
            if thermal.length(pos) < self.radius:
                return thermal
        return None

    def normalize_pos(self, drone):
        if drone.pos.x < 0:
            drone.pos.x += self.area_size
        if drone.pos.x > self.area_size:
            drone.pos.x -= self.area_size
        if drone.pos.y < 0:
            drone.pos.y += self.area_size
        if drone.pos.y > self.area_size:
            drone.pos.y -= self.area_size

    def inform_drone(self, drone):
        for thermal in self.thermals:
            drone.add_thermal(thermal)

    def inform_of_nearest(self, drone):
        nearest = 0
        min_dist = 100000000
        for i, thermal in enumerate(self.thermals):
            dist = thermal.length(drone.pos)
            # thermal.print_elements()
            if dist < min_dist:
                min_dist = dist
                nearest = i
        drone.add_thermal(self.thermals[nearest])


class Swarm(object):
    def __init__(self, size=1, height=10, thermals=100, area_size=1000, sigma=45):
        self.size = size
        self.start_height = height
        self.drones = [uav.UAV(0, 0, self.start_height) for _ in range(size)]
        self.v_falling = -0.1
        self.v_rising = 0.2
        self.set_norm_velocity(norm=1)
        self.start_velocity = uav.Vector(self.random_velocity(), self.random_velocity(), self.v_falling)
        self.set_velocity(self.start_velocity)
        self.min_height = 0.5 * height
        self.max_thermal_height = self.start_height - 1
        self.env = Environment(thermals, area_size=area_size)
        self.sigma = sigma
        for drone in self.drones:
            # self.env.inform_drone(drone)
            self.env.inform_of_nearest(drone)
            drone.area_size = area_size

    def set_velocity(self, velocity):
        for drone in self.drones:
            drone.set_velocity(velocity)

    def set_norm_velocity(self, norm):
        for drone in self.drones:
            drone.v_xy_norm = norm

    @staticmethod
    def random_velocity():
        return (random.random()*2) - 1

    def fly(self, steps=200000):
        step_length = 0.5
        alpha = 0
        v_cum = 0
        for ii in range(0, steps):
            for drone in self.drones:
                drone.random_fly = True
                # drone.pos.print_elements()
                thermal = self.env.get_thermal(drone.pos)
                if thermal:
                    drone.v.z = self.v_rising
                    if thermal is not None:
                        if not drone.is_known(thermal):
                            drone.add_thermal(thermal)
                    if drone.height() < self.min_height and self.env.in_middle(drone.pos):
                        drone.v.x = 0
                        drone.v.y = 0
                        drone.random_fly = False
                    if drone.height() < self.start_height and self.env.in_middle(drone.pos):
                        drone.random_fly = False
                else:
                    drone.v.z = self.v_falling
                    if drone.height() < self.min_height:
                        drone.go_to_nearest_thermal()
                        drone.random_fly = False
                if ii % 100 == 0 and drone.random_fly:
                    # drone.v.x = self.random_velocity()
                    # drone.v.y = self.random_velocity()
                    drone.gaussian_direction(alpha, self.sigma)
                drone.fly(step_length)
                self.env.normalize_pos(drone)
                v_cum += drone.v.x
        v_cum /= steps
        print(v_cum)
