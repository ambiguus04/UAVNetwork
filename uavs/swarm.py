from . import uav
import math
import random


class Thermal(uav.Vector):
    def length(self, end=None):
        if not end:
            end = Thermal()
        return math.sqrt((end.x - self.x)**2 + (end.y - self.y)**2)


class Environment(object):
    def __init__(self, size=1):
        self.thermals = [Thermal(20, 20, 0) for i in range(size)]
        self.radius = 1
        self.close = 0.5
        self.area_size = 1000

    def in_distance(self, pos, dist):
        for thermal in self.thermals:
            if thermal.length(pos) <= dist:
                return True
        return False

    def in_thermal(self, pos):
        return self.in_distance(pos, self.radius)

    def in_middle(self, pos):
        return self.in_distance(pos, self.close)

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

class Swarm(object):
    def __init__(self, size=1, height=10):
        self.size = size
        self.start_height = height
        self.drones = [uav.UAV(0, 0, self.start_height) for i in range(size)]
        self.v_falling = -0.1
        self.v_rising = 0.2
        self.set_norm_velocity(norm=1)
        self.start_velocity = uav.Vector(self.random_velocity(), self.random_velocity(), self.v_falling)
        self.set_velocity(self.start_velocity)
        self.min_height = 6
        self.alarm_height = 5
        self.max_thermal_height = self.start_height - 1
        self.env = Environment()
        for drone in self.drones:
            self.env.inform_drone(drone) #informuje o wszystkich

    def set_velocity(self, velocity):
        for drone in self.drones:
            drone.set_velocity(velocity)

    def set_norm_velocity(self, norm):
        for drone in self.drones:
            drone.v_xy_norm = norm

    @staticmethod
    def random_velocity():
        return (random.random()*2) - 1

    def fly(self, steps=3000):
        step_length = 0.1
        alpha = 0
        for ii in range(0, steps):
            for drone in self.drones:
                drone.random_fly = True
                # drone.pos.print_elements()
                print("pos ({:.2f}, {:.2f}, {:.2f}), v ({:.2f}, {:.2f}, {:.2f})"
                      .format(drone.pos.x, drone.pos.y, drone.pos.z, drone.v.x, drone.v.y, drone.v.z))
                if self.env.in_thermal(drone.pos): #w kominie
                    drone.v.z = self.v_rising
                    print("w kominie")
                    if drone.height() < self.min_height and self.env.in_middle(drone.pos):
                        drone.v.x = 0
                        drone.v.y = 0
                        drone.random_fly = False
                        print("tylko w gore")
                    if drone.height() < self.start_height and self.env.in_middle(drone.pos):
                        drone.random_fly = False
                else:
                    drone.v.z = self.v_falling
                    if drone.height() < self.min_height:
                        drone.go_to_nearest_thermal()
                        drone.random_fly = False
                        print("idziemy na thermal")
                if drone.height() >= self.start_height:
                    print("wyszlismy")
                if ii % 100 == 0 and drone.random_fly:
                    # drone.v.x = self.random_velocity()
                    # drone.v.y = self.random_velocity()
                    drone.gaussian_direction(alpha)
                drone.fly(step_length)
                self.env.normalize_pos(drone)
