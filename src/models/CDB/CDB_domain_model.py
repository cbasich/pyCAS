import os, sys, time, json, pickle, random

from collections import defaultdict
from IPython import embed
import numpy as np
import itertools as it

current_file_path = os.path.dirname(os.path.realpath(__file__))
sys.path.append(os.path.join(current_file_path, '..', '..'))

from CDB.CDB_helper import CampusDeliveryBotHelper
from scripts.utils import FVI

DOMAIN_PATH = os.path.join(current_file_path, '..', '..', '..', 'domains', 'CDB')
MAP_PATH = os.path.join(DOMAIN_PATH, 'maps')
PARAM_PATH = os.path.join(DOMAIN_PATH, 'params')

DIRECTIONS = [(1,0),(0,-1),(-1,0),(0,1)]
MAPPING = {(-1,0): 'NORTH', (0,1) : 'EAST', (1,0) : 'SOUTH', (0,-1): 'WEST'}
REV_MAPPING = {'NORTH': (-1,0), 'EAST': (0,1), 'SOUTH': (1,0), 'WEST': (0,-1)}

def read_gw_map(filename):
    grid = []
    with open(filename,'r+') as f:
        for line in f:
            grid.append(line.strip().split(' '))
    return grid

class DeliveryBotDomain():
    def __init__(self, filename, init, destination, gamma=1.0):
        self.gamma = gamma
        self.grid = read_gw_map(os.path.join(MAP_PATH, filename))
        self.rows = len(self.grid)
        self.cols = len(self.grid[0])

        self.kappa = {}
        try:
            self.kappa = pickle.load( open( os.path.join(PARAM_PATH,'kappa.pkl'), mode='rb'), encoding='bytes')
        except Exception:
            pass

        self.init = None

        self.actions = DIRECTIONS + ['cross', 'open', 'wait']
        self.states, self.goals = self.generate_states(init, destination)
        self.regions = self.generate_regions()
        self.transitions = self.generate_transitions()
        self.costs = self.generate_costs()

        self.check_validity()
        self.helper = CampusDeliveryBotHelper(self)

    def generate_states(self, init, destination):
        S = set(it.product(range(self.rows), range(self.cols)))

        states, goals = set(), set()

        for s in S:
            x,y = s[0],s[1]
            if self.grid[x][y] == 'S':
                states.add(s)
                init = s
                if s not in self.kappa.keys():
                    self.kappa[s] = {}
                    for a in self.actions:
                        self.kappa[s][a] = 3
            elif self.grid[x][y] == 'C':
                tmp_states = set(it.product([x], [y], list(REV_MAPPING.keys()), ['empty', 'light', 'busy']))
                states = states.union(tmp_states)
                for tmp in tmp_states:
                    if tmp not in self.kappa.keys():
                        self.kappa[tmp] = {}
                        for a in self.actions:
                            if a == 'wait':
                                self.kappa[tmp][a] = 3
                            else:
                                self.kappa[tmp][a] = 1
            elif self.grid[x][y] == 'L':
                tmp_states = set(it.product([x], [y], list(REV_MAPPING.keys()), ['light-open', 'light-closed']))
                states = states.union(tmp_states)
                for tmp in tmp_states:
                    if tmp not in self.kappa.keys():
                        self.kappa[tmp] = {}
                        for a in self.actions:
                            if 'closed' in tmp[3] and (a == REV_MAPPING[tmp[2]] or a == 'open'):
                                self.kappa[tmp][a] = 1
                            else:
                                self.kappa[tmp][a] = 3
            elif self.grid[x][y] == 'M':
                tmp_states = set(it.product([x], [y], list(REV_MAPPING.keys()), ['medium-open', 'medium-closed']))
                states = states.union(tmp_states)
                for tmp in tmp_states:
                    if tmp not in self.kappa.keys():
                        self.kappa[tmp] = {}
                        for a in self.actions:
                            if 'closed' in tmp[3] and (a == REV_MAPPING[tmp[2]] or a == 'open'):
                                self.kappa[tmp][a] = 1
                            else:
                                self.kappa[tmp][a] = 3
            elif self.grid[x][y] == 'H':
                tmp_states = set(it.product([x], [y], list(REV_MAPPING.keys()), ['heavy-open', 'heavy-closed']))
                states = states.union(tmp_states)
                for tmp in tmp_states:
                    if tmp not in self.kappa.keys():
                        self.kappa[tmp] = {}
                        for a in self.actions:
                            if 'closed' in tmp[3] and (a == REV_MAPPING[tmp[2]] or a == 'open'):
                                self.kappa[tmp][a] = 1
                            else:
                                self.kappa[tmp][a] = 3
            elif self.grid[x][y] == '.':
                states.add(s)
                if s not in self.kappa.keys():
                    self.kappa[s] = {}
                    for a in self.actions:
                        self.kappa[s][a] = 3
            elif not self.grid[x][y] == 'X':
                if init == self.grid[x][y]:
                    self.init = s
                elif destination == self.grid[x][y]:
                    goals.add(s)

                states.add(s)
                if s not in self.kappa.keys():
                    self.kappa[s] = {}
                    for a in self.actions:
                        self.kappa[s][a] = 3

        return list(states), list(goals)

    def generate_regions(self):
        regions = {}
        for s in self.states:
            if s[0] <= 14 and s[1] == 14:
                regions[(s[0], s[1])] = 'r1'

            elif s[0] == 15:
                if s[1] < 14:
                    regions[(s[0], s[1])] = 'r2'
                else:
                    regions[(s[0], s[1])] = 'r3'

            elif s[0] <= 14 and s[1] < 14:
                regions[(s[0], s[1])] = 'b1'

            elif s[0] <= 14 and s[1] > 14:
                regions[(s[0], s[1])] = 'b2'

            elif s[0] >= 16:
                regions[(s[0], s[1])] = 'b3'

        return regions


    def generate_costs(self):
        C = np.array([[1.0 for a in range(len(self.actions))] 
                           for s in range(len(self.states))])

        for goal in self.goals:
            C[self.states.index(goal)] *= 0.0

        return C


    def generate_transitions(self):
        T = np.array([[[0.0 for sp in range(len(self.states))]
                            for a in range(len(self.actions))]
                            for s in range(len(self.states))])

        for s, state in enumerate(self.states):
            for a, action in enumerate(self.actions):

                if state in self.goals: # Check for goal condition to self-loop
                    T[s][a][s] = 1.0

                for sp, statePrime in enumerate(self.states):

                    if action in DIRECTIONS:
                        T[s][a][sp] = self.move(state, action, statePrime)
                        
                    elif action == 'open':
                        T[s][a][sp] = self.open(state, statePrime)

                    elif action == 'wait':
                        T[s][a][sp] = self.wait(state, statePrime)

                    elif action == 'cross':
                        T[s][a][sp] = self.cross(state, statePrime)

                if np.sum(T[s][a]) == 0.0:
                    T[s][a][s] = 1.0

        return T


    def move(self, state, action, statePrime):
        xp, yp = state[0] + action[0], state[1] + action[1]

        if len(state) > 2 and ('closed' in state[3] or state[3] in ['empty', 'light', 'heavy']):
            if MAPPING[action] == state[2]:
                return 1.0 if state == statePrime else 0.0
            else:
                return 1.0 if (xp == statePrime[0] and yp == statePrime[1]) else 0.0

        if (statePrime == (xp, yp) or
            statePrime == (xp, yp, MAPPING[action], 'light-closed') or
            statePrime == (xp, yp, MAPPING[action], 'medium-closed') or
            statePrime == (xp, yp, MAPPING[action], 'heavy-closed')):
            return 1.0

        elif (statePrime == (xp, yp, MAPPING[action], 'empty') or
             statePrime == (xp, yp, MAPPING[action], 'light') or
             statePrime == (xp, yp, MAPPING[action], 'busy')):
            return 1.0/3.0

        return 0.0

    def cross(self, state, statePrime):
        if len(state) > 2 and state[3] in ['empt', 'light', 'heavy']:
            xp, yp = state[0] + REV_MAPPING[state[2]][0], state[1] + REV_MAPPING[state[2]][1]

            return 1.0 if (statePrime[0] == xp and statePrime[1] == yp) else 0.0

        return 1.0 if state == statePrime else 0.0

    def open(self, state, statePrime):
        if len(state) == 2 or not 'closed' in state[3]:
            return 1.0 if state == statePrime else 0.0

        elif (len(statePrime) > 2 and 
             state[0] == statePrime[0] and
             state[1] == statePrime[1] and
             state[2] == statePrime[2]):
            if ((state[3] == 'light-closed' and statePrime[3] == 'light-open') or
                (state[3] == 'medium-closed' and statePrime[3] == 'medium-open') or
                (state[3] == 'heavy-closed' and statePrime[3] == 'heavy-open')):
                return 1.0

        return 0.0


    def wait(self, state, statePrime):
        if (len(state) > 2 and state[3] in ['empty', 'light', 'busy'] and
           len(statePrime) > 2 and statePrime[3] in ['empty', 'light', 'busy'] and
           statePrime[0] == state[0] and statePrime[1] == state[1] and statePrime[2] == state[2]):

            if state[3] == 'empty':
                if statePrime[3] == 'empty':
                    return 0.75
                elif statePrime[3] == 'light':
                    return 0.25

            elif state[3] == 'light':
                if statePrime[3] == 'empty' or statePrime[3] == 'busy':
                    return 0.25

                elif statePrime[3] == 'light':
                    return 0.5

            elif state[3] == 'busy':
                if statePrime[3] == 'light':
                    return 0.25

                elif statePrime[3] == 'busy':
                    return 0.75

        return 0.0

    def check_validity(self):
        for s in range(len(self.states)):
            for a in range(len(self.actions)):

                if round(np.sum(self.transitions[s][a]),3) != 1.0:
                    print("Error @ state " + str(self.states[s]) + " and action " + str(self.actions[a]))
                    embed()
                    quit()