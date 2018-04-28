from uavs import *
import argparse

parser = argparse.ArgumentParser()
parser.add_argument("--thermals", type=int, dest='thermals', default=100)
parser.add_argument("--area", type=int, dest='area_size', default=1000)
parser.add_argument("--sigma", type=int, dest='sigma', default=45)
args = parser.parse_args()
s = swarm.Swarm(thermals=args.thermals, area_size=args.area_size, sigma=args.sigma)
s.fly()
