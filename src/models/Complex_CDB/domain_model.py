import os, sys, time, json, pickle, random

from collections import defaultdict
from IPython import embed
import numpy as np
import itertools as it

current_file_path = os.path.dirname(os.path.realpath(__file__))
sys.path.append(os.path.join(current_file_path, '..', '..'))

from Complex_CDB.domain_helper import CampusDeliveryBotHelper
from scripts.utils import FVI

DOMAIN_PATH = os.path.join(current_file_path, '..', '..', '..', 'domains', 'CDB')
MAP_PATH = os.path.join(DOMAIN_PATH, 'maps')
PARAM_PATH = os.path.join(DOMAIN_PATH, 'params')

DIRECTIONS = [(1,0),(0,-1),(-1,0),(0,1)]
MAPPING = {(-1,0): 'NORTH', (0,1) : 'EAST', (1,0) : 'SOUTH', (0,-1): 'WEST'}
REV_MAPPING = {'NORTH': (-1,0), 'EAST': (0,1), 'SOUTH': (1,0), 'WEST': (0,-1)}

"""
        TODOS:

                - Change existing naming convention.
                    -- empty, light, busy   -->     traffic_none, traffic_light, traffic_heavy
                    -- light, medium, heavy -->     door_size_small, door_size_medium, door_size_large
                    -- low, high            -->     visibility_low, visibility_high

                - Add new unused features
                    -- Local features
                        -- Doors: (door_type_push, door_type_pull)

                    -- Global features
                        -- Outside condition: (daytime, nighttime), (rain, clear)

                - Update feedback rules
                    -- daytime + clear + visibility_high    -->    yes
                    -- daytime + clear + visibility_low     -->    yes for traffic_none, traffic_light
                    -- nighttime + clear + visibility_high  -->    yes for traffic_none, traffic_light
                    -- nighttime + clear + visibility_low   -->    yes for traffic_none
                    -- rain + anything                      -->    no

                    -- door_type_pull + anything            -->    no
"""


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
        with open(os.path.join(MAP_PATH, 'map_info.json')) as F:
            self.map_info = json.load(F)
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
        self.transitions = self.generate_transitions()
        self.costs = self.generate_costs()

        self.check_validity()
        self.helper = CampusDeliveryBotHelper(self)

    def generate_states(self, init, destination):
        S = set(it.product(range(self.rows), range(self.cols)))

        used_features = open(os.path.join(PARAM_PATH, "used_features.txt")).readline().split(",")

        print(used_features)

        states, goals = set(), set()

        for s in S:
            x,y = s[0],s[1]

            if self.grid[x][y] == 'C':
                features = []
                if 'visibility' in used_features:
                    features.append('visibility_' + self.map_info[str((x,y))]['visibility'])
                if 'street_type' in used_features:
                    features.append('street_type_' + self.map_info[str((x,y))]['street_type'])

                if len(features) > 0:
                    tmp_states = set(it.product([x], [y], ['daytime', 'nighttime'], ['clear', 'rainy'], 
                        list(REV_MAPPING.keys()), ['traffic_none', 'traffic_light', 'traffic_heavy'], features))
                else:
                    tmp_states = set(it.product([x], [y], ['daytime', 'nighttime'], ['clear', 'rainy'], 
                        list(REV_MAPPING.keys()), ['traffic_none', 'traffic_light', 'traffic_heavy']))
                states = states.union(tmp_states)
                for tmp in tmp_states:
                    if tmp not in self.kappa.keys():
                        self.kappa[tmp] = {}
                        for a in self.actions:
                            if a == 'cross':
                                self.kappa[tmp][a] = 1
                            else:
                                self.kappa[tmp][a] = 3
            elif self.grid[x][y] == 'D':
                features = []
                if 'door_size' in used_features:
                    features.append('door_size_' + self.map_info[str((x,y))]['door_size'])
                if 'door_color' in used_features:
                    features.append('door_color_' + self.map_info[str((x,y))]['door_color'])
                if 'door_type' in used_features:
                    features.append('door_type_' + self.map_info[str((x,y))]['door_type'])

                if len(features) > 0:
                    tmp_states = set(it.product([x], [y], list(REV_MAPPING.keys()), ['door_open', 'door_closed'], features))
                else:
                    tmp_states = set(it.product([x], [y], list(REV_MAPPING.keys()), ['door_open', 'door_closed']))
                states = states.union(tmp_states)
                for tmp in tmp_states:
                    if tmp not in self.kappa.keys():
                        self.kappa[tmp] = {}
                        for a in self.actions:
                            if 'closed' in tmp[3] and a == 'open':
                                self.kappa[tmp][a] = 1
                            else:
                                self.kappa[tmp][a] = 3
            elif self.grid[x][y] == '.':
                tmp_states = set(it.product([x], [y], ['daytime', 'nighttime'], ['clear', 'rainy']))
                states = states.union(tmp_states)
                for tmp in tmp_states:
                    if tmp not in self.kappa.keys():
                        self.kappa[tmp] = {}
                        for a in self.actions:
                            self.kappa[tmp][a] = 3
            elif not self.grid[x][y] == 'X':
                tmp_states = set(it.product([x], [y], ['daytime', 'nighttime'], ['clear', 'rainy']))
                states = states.union(tmp_states)

                if init == self.grid[x][y]:
                    self.init = random.choice(tuple(tmp_states))
                elif destination == self.grid[x][y]:
                    goals = tmp_states

                for tmp in tmp_states:
                    if tmp not in self.kappa.keys():
                        self.kappa[tmp] = {}
                        for a in self.actions:
                            self.kappa[tmp][a] = 3

        return list(states), list(goals)

    def generate_costs(self):
        C = np.ones((len(self.states), len(self.actions)))
        for goal in self.goals:
            C[self.states.index(goal)] *= 0.0
        return C

    def generate_transitions(self):
        T = np.zeros((len(self.states), len(self.actions), len(self.states)))

        for s, state in enumerate(self.states):
            for a, action in enumerate(self.actions):

                if state in self.goals: # Check for goal condition to self-loop
                    T[s][a][s] = 1.0
                    continue

                for sp, statePrime in enumerate(self.states):
                    if statePrime[2] != state[2] or statePrime[3] != state[3]:
                        continue

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

        if len(state) > 4 and (state[5] in ['door_closed', 'traffic_none', 'traffic_light', 'traffic_heavy']):
            if MAPPING[action] == state[4]:
                return 1.0 if state == statePrime else 0.0
            else:
                return 1.0 if (xp == statePrime[0] and yp == statePrime[1]) else 0.0

        if len(statePrime) == 4 and (statePrime[0] == xp and statePrime[1] == yp):
            return 1.0

        elif statePrime[:6] == (xp, yp, state[2], state[3], MAPPING[action], 'door_closed'):
            return 1.0

        elif (statePrime[:6] == (xp, yp, state[2], state[3], MAPPING[action], 'traffic_none') or
              statePrime[:6] == (xp, yp, state[2], state[3], MAPPING[action], 'traffic_light') or
              statePrime[:6] == (xp, yp, state[2], state[3], MAPPING[action], 'traffic_heavy')):
            return 1.0/3.0

        return 0.0

    def cross(self, state, statePrime):
        if len(state) > 4 and state[5] in ['traffic_none', 'traffic_light', 'traffic_heavy']:
            xp, yp = state[0] + REV_MAPPING[state[4]][0], state[1] + REV_MAPPING[state[4]][1]

            return 1.0 if (statePrime[0] == xp and statePrime[1] == yp) else 0.0

        return 1.0 if state == statePrime else 0.0

    def open(self, state, statePrime):
        if len(state) < 5 or not 'closed' in state[5]:
            return 1.0 if state == statePrime else 0.0
        elif len(statePrime) > 5 and state[:5] == statePrime[:5] and state[5] == 'door-closed' and statePrime[5] == 'door-open':
            return 1.0

        return 0.0

    def wait(self, state, statePrime):
        if (len(state) > 5 and state[5] in ['traffic_none', 'traffic_light', 'traffic_heavy'] and
            len(statePrime) > 5 and statePrime[5] in ['traffic_none', 'traffic_light', 'traffic_heavy'] and
            state[:5] == statePrime[:5]):

            if state[5] == 'traffic_none':
                if statePrime[5] == 'traffic_none':
                    return 0.75
                elif statePrime[5] == 'traffic_light':
                    return 0.25

            elif state[5] == 'traffic_light':
                if statePrime[5] == 'traffic_none' or statePrime[5] == 'traffic_heavy':
                    return 0.25
                elif statePrime[5] == 'traffic_light':
                    return 0.5

            elif state[5] == 'traffic_heavy':
                if statePrime[5] == 'traffic_light':
                    return 0.25
                elif statePrime[5] == 'traffic_heavy':
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