import os, sys, time, json, pickle, random

from collections import defaultdict
from IPython import embed
import numpy as np
import itertools as it

current_file_path = os.path.dirname(os.path.realpath(__file__))
sys.path.append(os.path.join(current_file_path, '..', '..'))



DOMAIN_PATH = os.path.join(current_file_path, '..', '..', '..', 'domains', 'CDB')
MAP_PATH = os.path.join(DOMAIN_PATH, 'maps')


def read_gw_map(filename):
    grid = []
    with open(filename,'r+') as f:
        for line in f:
            grid.append(line.strip().split(' '))
    return grid

grid = read_gw_map('/home/willcoe/pyCAS/src/pyCAS/domains/CDB/maps/campus_1.txt')
print(grid)
print(len(grid))
print(len(grid[0]))