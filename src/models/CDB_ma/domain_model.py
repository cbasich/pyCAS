import os, sys, time, json, pickle, random

from collections import defaultdict
from IPython import embed
import numpy as np
import pandas as pd
import itertools as it

current_file_path = os.path.dirname(os.path.realpath(__file__))
sys.path.append(os.path.join(current_file_path, '..', '..'))

from CDB_ma.domain_helper import Helper
from scripts.utils import FVI

DOMAIN_PATH = os.path.join(current_file_path, '..', '..', '..', 'domains', 'CDB_ma')
MAP_PATH = os.path.join(DOMAIN_PATH, 'maps')

DIRECTIONS = [(1,0),(0,-1),(-1,0),(0,1)]
MAPPING = {(-1,0): 'NORTH', (0,1) : 'EAST', (1,0) : 'SOUTH', (0,-1): 'WEST'}
REV_MAPPING = {'NORTH': (-1,0), 'EAST': (0,1), 'SOUTH': (1,0), 'WEST': (0,-1)}

def read_gw_map(filename):
    grid = []
    with open(filename,'r+') as f:
        for line in f:
            grid.append(line.strip().split(' '))
    return grid

class DomainModel():
    def __init__(self, filename, gamma=1.0):
        self.gamma = gamma
        self.grid = read_gw_map(os.path.join(MAP_PATH, filename))
        self.rows = len(self.grid)
        self.cols = len(self.grid[0])

        self.map_info = json.load(open(os.path.join(MAP_PATH, 'map_info.json'), mode='r'))
        self.helper = Helper(self)

        self.timeofday = np.random.choice(['morning', 'midday', 'afternoon'])

        self.actions = DIRECTIONS + ['cross', 'open', 'wait']        
        self.states, self.offices, self.init, self.goals = None, None, None, None
        self.generate_states()
        
        self.transitions, self.costs = None, None
        # self.compute_transitions()
        # self.compute_costs()
        # self.check_validity()

    def generate_states(self,):
        S = set(it.product(range(self.rows), range(self.cols)))
        states= set()
        offices = set()

        for s in S:
            x,y = s[0],s[1]
            if self.grid[x][y] == 'C':
                f1 = self.helper.map_info[str((x,y))]['visibility']
                states = states.union(set(it.product([x], [y], list(REV_MAPPING.keys()), ['empty', 'light', 'busy'], [f1])))
            elif self.grid[x][y] == 'D':
                f1 = self.helper.map_info[str((x,y))]['doortype']
                f2 = self.helper.map_info[str((x,y))]['doorsize']
                states = states.union(set(it.product([x], [y], list(REV_MAPPING.keys()), ['door-open', 'door-closed'], [f1], [f2])))
            elif self.grid[x][y] == '.':
                states.add(s)
            elif not self.grid[x][y] == 'X':
                states.add(s)
                offices.add(s)

        self.states = list(states)
        self.offices = offices

    def set_init(self, start):
        loc = np.where(np.array(self.grid) == start)
        self.init = (loc[0][0], loc[1][0])

    def set_goal(self, destination):
        loc = np.where(np.array(self.grid) == destination)
        self.goals = [(loc[0][0], loc[1][0])]

    def set_random_task(self):
        self.init = random.choice(tuple(self.offices))
        goal = random.choice(tuple(self.offices))
        while goal == self.init:
            goal = random.choice(tuple(self.offices))
        self.goals = [goal]
        self.compute_transitions()
        self.compute_costs()

    def compute_costs(self):
        C = np.array([[1.0 for a in range(len(self.actions))] 
                           for s in range(len(self.states))])

        for goal in self.goals:
            C[self.states.index(goal)] *= 0.0

        self.costs = C

    def compute_transitions(self):
        T = np.array([[[0.0 for sp in range(len(self.states))]
                            for a in range(len(self.actions))]
                            for s in range(len(self.states))])

        for s, state in enumerate(self.states):
            for a, action in enumerate(self.actions):
                if state in self.goals:
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

        self.transitions = T


    def move(self, state, action, statePrime):
        xp, yp = state[0] + action[0], state[1] + action[1]

        if len(state) > 2 and ('closed' in state[3] or state[3] in ['empty', 'light', 'busy']):
            if MAPPING[action] == state[2]:
                return 1.0 if state == statePrime else 0.0
            else:
                return 1.0 if (xp == statePrime[0] and yp == statePrime[1]) else 0.0

        if (statePrime == (xp, yp)):
            return 1.0

        elif statePrime[:4] == (xp, yp, MAPPING[action], 'door-closed'):
            return 1.0

        elif (statePrime[:4] == (xp, yp, MAPPING[action], 'empty') or
             statePrime[:4] == (xp, yp, MAPPING[action], 'light') or
             statePrime[:4] == (xp, yp, MAPPING[action], 'busy')):
            return 1.0/3.0

        return 0.0

    def cross(self, state, statePrime):
        if len(state) > 2 and state[3] in ['empty', 'light', 'busy']:
            xp, yp = state[0] + REV_MAPPING[state[2]][0], state[1] + REV_MAPPING[state[2]][1]

            return 1.0 if (statePrime[0] == xp and statePrime[1] == yp) else 0.0

        return 1.0 if state == statePrime else 0.0

    def open(self, state, statePrime):
        if len(state) == 2 or not 'closed' in state[3]:
            return 1.0 if state == statePrime else 0.0

        elif len(statePrime) > 2 and state[:3] == statePrime[:3]:
            if state[3] == 'door-closed' and statePrime[3] == 'door-open':
                return 1.0

        return 0.0

    def wait(self, state, statePrime):
        if (len(state) > 2 and state[3] in ['empty', 'light', 'busy'] and
            len(statePrime) > 2 and statePrime[3] in ['empty', 'light', 'busy'] and
            state[:3] == statePrime[:3]):

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

    def generate_successor(self, state, action):
        """
            params:
                state - The state we are generating a successor for.
                action - The action we are generating a successor for.

            returns:
                successor - The successor state that the agent arrives in
                            when taking 'action' in 'state', as determined
                            by the transition function. 
        """
        s = self.states.index(state)
        a = self.actions.index(action)

        rand = np.random.uniform()
        thresh = 0.0

        for sp in range(len(self.states)):
            thresh += self.transitions[s][a][sp]
            if rand <= thresh:
                return self.states[sp]