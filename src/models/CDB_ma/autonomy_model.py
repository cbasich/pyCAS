import os, sys, time, random, pickle

import numpy as np
import itertools as it

from copy import deepcopy
from collections import defaultdict
from threading import Thread
from IPython import embed

current_file_path = os.path.dirname(os.path.realpath(__file__))
sys.path.append(os.path.join(current_file_path, '..'))

DOMAIN_PATH = os.path.join('..', '..', 'domains')
PARAM_PATH = os.path.join(DOMAIN_PATH, 'CDB_icra', 'params')

class AutonomyModel():
    def __init__(self, DM, L):
        self.DM = DM
        self.L = L
        self.kappa = self.generate_kappa()


    def generate_kappa(self):
        kappa = {}
        for s, state in enumerate(self.DM.states):
            kappa[state] = {}
            for a, action in enumerate(self.DM.actions):
                if str((state[0], state[1])) not in self.DM.map_info.keys():
                    kappa[state][action] = 3
                elif ((state[3] == 'door-closed' and action == 'open' )
                    or (state[3] in ['empty','light','busy'] and action == 'cross')):
                    kappa[state][action] = 1
                else:
                    kappa[state][action] = 3
        return kappa

    def update_kappa(self, state, action, level):
        self.kappa[state][action] = level
        

    def mu(self, state, action):
        return 0.0
        # return 0.25 if state[1] != action[1] else 0.0