import os, sys, time, random, pickle

import numpy as np
from IPython import embed

current_file_path = os.path.dirname(os.path.realpath(__file__))
sys.path.append(os.path.join(current_file_path, '..', '..'))

from scripts.utils import build_gam

DOMAIN_PATH = os.path.join(current_file_path, '..', '..', '..', 'domains', 'CDB')
FEEDBACK_PATH = os.path.join(DOMAIN_PATH, 'feedback')
PARAM_PATH = os.path.join(DOMAIN_PATH, 'params')

def build_gams():
    build_gam('CDB', 'cross')
    build_gam('CDB', 'open')

def load_gams():
    cross_GAM = pickle.load( open( os.path.join(PARAM_PATH, 'cross_gam.pkl'), mode='rb'), encoding='bytes')
    open_GAM = pickle.load( open( os.path.join(PARAM_PATH, 'open_gam.pkl'), mode='rb'), encoding='bytes')

    cross_GAM_map = pickle.load( open( os.path.join(PARAM_PATH, 'cross_gam_map.pkl'), mode='rb'), encoding='bytes')
    open_GAM_map = pickle.load( open( os.path.join(PARAM_PATH, 'open_gam_map.pkl'), mode='rb'), encoding='bytes')

    return cross_GAM, open_GAM, cross_GAM_map, open_GAM_map

class CampusDeliveryBotHelper():
    def __init__(self, DM):
        self.DM = DM
        build_gams()
        self.cross_GAM, self.open_GAM, self.cross_GAM_map, self.open_GAM_map = load_gams()

    def extract_state_features(self, state):
        regions = self.DM.regions
        f1 = regions[(state[0], state[1])]
        f2 = state[3]
        f = [f1, f2]
        return f

    def predict(self, state, action, level):
        if len(state) == 2 or 'open' in state[3]:
            return 1.
        features = [level] + self.extract_state_features(state)
        if action == 'open' and 'door' in state[3]:
            return self.open_GAM.predict_mu([[self.open_GAM_map[f] for f in features]])
        elif action == 'cross' and 'door' not in state[3]:
            return self.cross_GAM.predict_mu([[self.cross_GAM_map[f] for f in features]])
        else:
            return -1.

    def get_door_type(self, state):
        x, y = state[0], state[1]
        if self.DM.grid[x][y] == 'L':
            return 'light'
        elif self.DM.grid[x][y] == 'M':
            return 'medium'
        elif self.DM.grid[x][y] == 'H':
            return 'heavy'
        else:
            return