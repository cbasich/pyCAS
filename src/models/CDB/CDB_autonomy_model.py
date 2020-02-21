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
PARAM_PATH = os.path.join(DOMAIN_PATH, 'CDB', 'params')

class AutonomyModel():
    def __init__(self, DM, L):
        self.DM = DM
        self.L = L
        self.kappa = self.generate_kappa()

    def generate_kappa(self):
        kappa = None
        try:
            kappa = (pickle.load( open( os.path.join(PARAM_PATH, 'kappa.pkl'), mode='rb'), encoding='bytes'))
        except Exception:
            # We actually generate kappa while building DM to save some computation time so that
            # we don't have to iterate through all of S x A again
            kappa = self.DM.kappa
        return kappa

    def save_kappa(self):
        try:
           kappa_file = open( os.path.join(PARAM_PATH, 'kappa.pkl'), 'wb')
        except Exception:
            embed()
        pickle.dump(self.kappa, kappa_file, protocol=pickle.HIGHEST_PROTOCOL)
        kappa_file.close()

    def mu(self, state, action):
        return 0.25 if state[1] != action[1] else 0.0