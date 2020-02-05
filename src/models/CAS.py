import os, sys, time, random, pickle

import numpy as np
import itertools as it

from copy import deepcopy
from collections import defaultdict
from IPython import embed

current_file_path = os.path.dirname(os.path.realpath(__file__))
sys.path.append(os.path.join(current_file_path, '..'))

from utils.utils import FVI

PARAM_PATH = os.path.join('..', 'data', 'model parameters')


class CAS():
    def __init__(self, DM, AM, HM, persistence):
        self.DM = DM        # DM = < S, A, T, C, s_0, s_g >
        self.AM = AM        # AM = < L, kappa, mu >
        self.HM = HM        # HM = < Sigma, lambda, rho, tau, T_H >

        self.persistence = persistence

        self.states = self.generate_states()
        self.actions = self.generate_actions()
        self.init = (self.DM.init, self.AM.L[0])
        self.goals = list(it.product(self.DM.goal, self.AM.L))

        self.transitions = self.compute_transitions()
        self.transitions_base = self.transitions.copy()
        self.check_validity()
        self.costs = self.compute_costs()

        self.potential = self.compute_potential()

        self.pi = None
        self.state_map = None
        self.V = None
        self.Q = None

    def generate_states(self):
        return list(it.product(self.DM.states, self.AM.L))

    def generate_actions(self):
        return list(it.product(self.DM.actions, self.AM.L))

    def compute_transitions(self):
        T = np.array([[[0.0 for sp in range(len(self.states))]
                            for a in range(len(self.actions))]
                            for s in range(len(self.states))])

        for s, state in enumerate(self.states):
            s_dm = self.DM.states.index(state[0])
            for a, action in enumerate(self.actions):
                a_dm = self.DM.actions.index(action[0])

                # Make sure action level is allowed by autonomy profile
                if action[1] > self.AM.kappa[state][action[0]]:
                    T[s][a][s] = 1.0
                    continue

                for sp, statePrime in enumerate(self.states):
                    if statePrime[1] != action[1]:      # Can only go to states that agree with level
                        continue
                    sp_dm = self.DM.states.index(statePrime[0])

                    if action[1] == 0:      # If operating in *no autonomy* which always exists, follow T_H exactly
                        T[s][a][sp] = self.HM.T_H(state, action, statePrime)     # TODO: Double check that this is correct

                    else:                   # Init value to DM's transition value. Can only be *decreased* from this
                        T[s][a][sp] = self.DM.transitions[s_dm][a_dm][sp_dm]

                    T[s][a][sp] *= self.HM.tau()    # tau will encompass the feedback dynamics

                if np.sum(T[s][a]) == 0.0:
                    T[s][a][s] = 1.0

        return T

    def compute_costs(self):
        C = np.array([[0.0 for a in range(len(self.actions))]
                           for s in range(len(self.states))])

        for s, state in enumerate(self.states):
            if state in self.goals:
                continue

            s_dm = self.DM.states.index(state[0])
            for a, action in enumerate(self.actions):
                a_dm = self.DM.actions.index(action[0])

                # Add the domain cost first
                C[s][a] += self.DM.costs[s_dm][a_dm]

                # Add the utility of autonomy
                C[s][a] += self.AM.mu()

                # Add the human cost penalty
                C[s][a] += self.HM.rho()

        return C

    def compute_potential(self):
        return np.array([[[0.0 for l in self.AM.L]
                               for a in self.DM.actions]
                               for s in self.DM.states])

    def check_validity(self):
        for s in range(len(self.states)):
            for a in range(len(self.actions)):

                if round(np.sum(self.transitions[s][a]), 3) != 1.0:
                    print("Error @ state " + str(self.states[s]) + " and action " + str(self.actions[a]))
                    embed()
                    quit()

    def q(self, s, state, a, action, l):
        # Note about sbar: right now assuming that the level component of a bar-state does
        # not actually have any impact on transition dynamics. This is why we can just use 
        # the transitions for (s, 0). However, if it does make a difference, then in fact
        # kappa must be at the bar-state level, not just the regular state level. In which
        # case, we should in fact build kkappa in here rather than leaving it to the DM to 
        # save on computation. This would be less efficient in general but more *correct*.
        sbar = self.states.index((state, 0))
        abar = self.actions.index((self.DM.actions[a], l))
        T = self.transitions[sbar][abar]
        q = self.costs[sbar][abar] - self.AM.mu(0, l) + np.sum(np.array(self.V * T))
        return q

    def update_potential(self, s, state, a, action, L):
        X = np.array([self.q(s, state, a, action, l) for l in L])
        softmax = (np.exp(-1.0 * X)/np.sum(np.exp(-1.0 * X)))
        for l in range(len(L)):
            self.potential[s][a][L[l]] += self.persistence * softmax[l]
        self.potential[s][a] = np.clip(self.potential[s][a], a_min = 0, a_max = 1)

    def update_kappa(self):
        for s, state in enumerate(self.DM.states):
            for a, action in enumerate(self.DM.actions):
                level = self.AM.kappa[state][action]
                L = [level-1, level, level+1]
                self.update_potential(s, state, a, action, L)

                for level_index in np.argsort(-1.0 * np.array([self.potential[s][a][l] for l in L])):
                    if np.random.uniform() <= self.potential[s][a][L[level_index]]:
                        # TODO insert level update logic here
                        self.AM.kappa[state][action] = L[level]
                        self.potential[s][a][L[level]] = 0.0
                        break

    def solve(self):
        start_time = time.time()
        mdp_info = value_iteration(self)
        end_time = time.time()

        self.pi = mdp_info['pi']
        self.state_map = mdp_info['state map']
        self.V = mdp_info['V']
        self.Q = mdp_info['Q']

        return (end_time - start_time)

    def generate_successor(self, state, action):
        s = self.states.index(state)
        a = self.actions.index(action)

        rand = np.random.uniform()
        thresh = 0.0

        for sp in range(len(self.states)):
            thresh += self.transitions[s][a][sp]
            if rand <= thresh:
                return self.states[sp]

    def remove_transitions(self, state, action):
        s = self.states.index(state)

        for i in np.arange(1, len(self.AM.L)):
            a = self.actions.index((action, i))
            self.transitions[s][a] *= 0.0
            self.transitions[s][a][s] = 1.0

    def reset(self):
        self.transitions = self.transitions_base

if __name__ == "__main__":
    print("hello")