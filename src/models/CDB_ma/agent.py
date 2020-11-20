import os, sys, time, random, pickle, json
import numpy as np
import pandas as pd
import itertools as it

from IPython import embed
from sklearn import svm
from sklearn.pipeline import make_pipeline
from sklearn.preprocessing import OneHotEncoder, normalize, StandardScaler

current_file_path = os.path.dirname(os.path.realpath(__file__))
sys.path.append(os.path.join(current_file_path, '..'))

MAP_PATH = os.path.join(current_file_path, '..', '..', '..', 'domains', 'CDB_ma', 'maps')
AGENT_PATH = os.path.join(current_file_path, '..', '..', '..', 'domains', 'CDB_ma', 'agents')

class Agent():
    def __init__(self, ID):
        self.id = ID
        self.authority_epsilon = 0.0
        self.CAS = None


    def set_authority_epsilon(self, eps):
        self.authority_epsilon = eps


    def set_cas_model(self, model):
        self.CAS = model


    def _interface_with_human(self, state, action, info, timeofday):
        feedback = None
        if info['obstacle'] == 'door':
            if info['doortype'] == 'pull':
                feedback = 'no'
            else:
                if ((info['doorsize'] == 'small') 
                 or (info['doorsize'] == 'medium' and info['region'] in ['b1','b2'])):
                    feedback = 'yes'
                else:
                    feedback = 'no'

        elif info['obstacle'] == 'crosswalk':
            if state[3] == 'empty' or (info['visibility'] == 'high' and state[3] == 'light'):
                if timeofday in info['pedestrians']:
                    feedback = 'no'
                else:
                    feedback = 'yes'
            else:
                feedback = 'no'

        if np.random.uniform() <= self.authority_epsilon:
            feedback = ['yes', 'no'][np.random.randint(2)]

        return feedback


    def rollout(self, num_trials=1):
        initial_policy = self.CAS.pi
        initial_transition_matrix = self.CAS.transitions.copy()

        with open(os.path.join(MAP_PATH, 'map_info.json')) as f:
            map_info = json.load(f)

        # simulation_trace_file = open(os.path.join(OUTPUT_PATH, 'simulation_trace.txt'), mode='a+')
        # simulation_trace_file.write("BEGINNING EPISODE {}\n".format(episode))

        costs = []
        for trial in range(num_trials):
            # simulation_trace_file.write("BEGINNING TRIAL\n")

            #reset the policy in case it changed in prior trial.
            acting_policy = initial_policy
            state = self.CAS.init
            cost = 0

            while state not in self.CAS.goals:
                # print(state)
                domain_state = state[0]
                state_index = self.CAS.state_map[state]
                action = acting_policy[state_index]
                action_index = self.CAS.actions.index(action)
                cost += self.CAS.costs[state_index][action_index]

                if action[1] == 1 or action[1] == 2:
                    feedback = self._interface_with_human(domain_state, action, 
                                        map_info[str((domain_state[0], domain_state[1]))], self.CAS.DM.timeofday)
                    if trial == num_trials - 1:
                        self.CAS.HM._update_data(state, action, feedback)

                    if feedback == 'no':
                        self.CAS.remove_transition(state, action)
                        self.CAS.solve()
                        acting_policy = self.CAS.pi
                    elif feedback == 'yes':
                        state = (self.CAS.DM.generate_successor(domain_state, action[0]), state[1])
                    else:
                        state = self.CAS.generate_successor(state, action)
                else:
                    state = self.CAS.generate_successor(state, action)

            costs.append(cost)
            self.CAS.set_transitions(initial_transition_matrix)

        return costs

    def save(self):
        with open(os.path.join(AGENT_PATH, 'agent_{}.pkl'.format(self.id)), mode='wb') as f:
            pickle.dump(self, f, protocol=pickle.HIGHEST_PROTOCOL)