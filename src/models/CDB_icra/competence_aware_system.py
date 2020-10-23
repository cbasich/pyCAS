import os, sys, time, random, pickle

import numpy as np
import itertools as it

from copy import deepcopy
from collections import defaultdict
from IPython import embed

current_file_path = os.path.dirname(os.path.realpath(__file__))
sys.path.append(os.path.join(current_file_path, '..'))

from scripts.utils import FVI

PARAM_PATH = os.path.join('..', 'data', 'model parameters')


class CAS():
    """Competence-Aware System

    Params:
    -------
    DM - Domain model, an SSP <S, A, T, C, s_0, s_g>.
        S is a set of states representing the domain
        A is a set of actions available to the agent
        T is the transition function
        C is the cost function
        s_0 is the initial domain state
        s_g is the domain goal state

    AM - Autonomy model, <L, kappa, mu>
        L  is the set of levels of autonomy
        Kappa is the autonomy profile and maps (s, a) --> l
        mu is the autonomy utility and maps (s, a) --> real #s

    HM - Feedback model, <Sigma, lambda, rho, tau, T_H>
        Sigma is a set of predefind feedback signals
        lambda is the feedback profile and maps (s,a,sigma) --> [0,1]
        rho is the cost of human assistance and maps l --> real #s
        tau is the level-based transition augmentation
        T_H is the human transition function

    persistence - Real value in [0, 1] used for gated exploration
                  The higher the value, the more frequently the system
                  will explore new levels of autonomy. 
        
    """
    def __init__(self, DM, AM, HM, persistence):
        self.DM = DM        # DM = < S, A, T, C, s_0, s_g >
        self.AM = AM        # AM = < L, kappa, mu >
        self.HM = HM        # HM = < Sigma, lambda, rho, tau, T_H >

        self.persistence = persistence
        self.gamma = self.DM.gamma

        self._states = self.generate_states()
        self._actions = self.generate_actions()
        self._init, self._goals, self._transitions, self._costs = None, None, None, None
        self.set_init()
        self.set_goals()
        self.compute_transitions()
        self.compute_costs()

        self.transitions_base = self.transitions.copy()
        self.check_validity()

        self.flags = [[False for a in range(len(self.DM.actions))] for s in range(len(self.states))]
        self.potential = np.array([[[0.0 for l in self.AM.L] for a in self.DM.actions] for s in self.DM.states])

        self.pi = None
        self.state_map = None
        self.V = None
        self.Q = None
        self.solver = None


    def generate_states(self):
        """
            params:
                None

            returns:
                A list of CAS-states where each CAS-state is a tuple of a
                domain state s and a level of autonomy l, where l is the
                level that the previous action was performed in.
        """
        return list(it.product(self.DM.states, [3]))


    @property
    def states(self):
        return self._states


    @property
    def init(self):
        return self._init


    @property
    def goals(self):
        return self._goals


    def set_init(self):
        self._init = (self.DM.init, 3)


    def set_goals(self):
        self._goals = list(it.product(self.DM.goals, [3]))


    def generate_actions(self):
        """
            params:
                None

            returns:
                A list of CAS-actions where each CAS-action is a tuple
                of a domain action a and a level of autonomy l.
        """
        return list(it.product(self.DM.actions, self.AM.L))


    @property
    def actions(self):
        return self._actions

    
    def compute_transitions(self):
        """
            params:
                None

            returns:
                A 3d numpy array of size |self.states| * |self.actions| * |self.states|
                where each entry T[s][a][sp] is the probability of arriving in state sp
                when taking action a in state s. T is a function of the domain state 
                transition function as well as the feedback model's tau and T_H functions.
        """
        T = np.zeros((len(self.states), len(self.actions), len(self.states)))

        # L = len(self.AM.L)
        L = 1                                                   # Removing for testing to speed things up.

        for s, state in enumerate(self.DM.states):
            for l1 in range(L):
                s_bar = L * s + l1                              # Set index in T
                for a, action in enumerate(self.DM.actions):
                    for l2 in range(len(self.AM.L)):
                        a_bar = len(self.AM.L) * a + l2                      # Set index in T
                        if l2 > self.AM.kappa[state][action]:
                            T[s_bar][a_bar][s_bar] = 1.         # Disallow levels above kappa(s,a)
                            continue
                        for sp, statePrime in enumerate(self.DM.states):
                            for l3 in range(L):
                                # if l3 != l2:                  # State Prime level must match action level
                                #     continue
                                sp_bar = L * sp + l3            # Set index in T
                                if l2 == 0:
                                    if action in self.HM.flagged: 
                                        T[s_bar][a_bar][sp_bar] = self.HM.T_H[s][a][sp]
                                    else:
                                        T[s_bar][a_bar][sp_bar] = self.DM.transitions[s][a][sp]
                                else:
                                    T[s_bar][a_bar][sp_bar] = self.HM.tau(s, state, l1, a, action, l2, sp, statePrime)
                        if np.sum(T[s_bar][a_bar]) == 0.:
                            T[s_bar][a_bar][s_bar] = 1.
                        if np.sum(T[s_bar][a_bar]) != 1.:
                            embed()
        self._transitions = T


    @property
    def transitions(self):
        return self._transitions


    def set_transitions(self, transitions):
        self._transitions = transitions
    

    def T(self, s, a, sp):
        """

        """
        if sp is None:
            return self.transitions[s][a]
        else:
            return self.transitions[s][a][sp]


    def compute_costs(self):
        """
            params:
                None

            returns:
                A 2d numpy array of size |self.states| * |self.actions| where
                each entry C[s][a] is the cost of taking action a in state s.
                C is a function of the domain model's cost function as well as
                mu and rho.
        """
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
                C[s][a] += self.AM.mu(state, action)

                # Add the human cost penalty
                C[s][a] += self.HM.rho(state, action)
        self._costs = C


    @property
    def costs(self):
        return self._costs


    def C(self, s, a):
        return self.costs[s][a]


    def check_validity(self):
        """
            params: 
                None

            returns:
                None

            description:
                Checks to make sure that T is a proper probability distribution.
                If there is some (s,a) for which T[s][a] does not sum to one,
                enter embed for debugging purposes and then quit.
        """
        for s in range(len(self.states)):
            for a in range(len(self.actions)):

                if round(np.sum(self.transitions[s][a]), 3) != 1.0:
                    print("Error @ state " + str(self.states[s]) + " and action " + str(self.actions[a]))
                    embed()
                    quit()


    def q(self, state, s, action, a, l):
        """
            params:
                state - The state we are calculating the q-value for
                s - The index of state
                action - The action we are calculating the q-value for
                a - The index of action
                l - the level we are calculating the q-value for. 

            returns:
                q - The q-value for (s,a) performed at level l

            notes:
                Right now assuming that the level component of a bar-state does
                not actually have any impact on transition dynamics. This is why we can just use 
                the transitions for (s, 0). However, if it does make a difference, then in fact
                kappa must be at the bar-state level, not just the regular state level. In which
                case, we should in fact build kkappa in here rather than leaving it to the DM to 
                save on computation. This would be less efficient in general but more *correct*.
        """
        sbar = self.states.index((state, 3))
        abar = self.actions.index((self.DM.actions[a], l))
        T = self.transitions[sbar][abar]
        q = self.costs[sbar][abar] + np.sum(np.array(self.V * T))
        return q


    def update_potential(self, state, s, action, a, L):
        """
            params:
                state  - The state we are updating the potential of.
                s      - The index of state in self.states.
                action - The action we are updating the potential of.
                a      - The index of action in self.actions.
                L      - The levels of autonomy we are updting the potential for.

            returns:
                None

            description:
                Updates the potential of the level that action a is being formed in
                in state s, and all adjacent levels in self.AM.L. Potential update is
                the persistence times the softmax over the q values of q(s, a, l) for
                each l in L. 
        """ 
        X = np.array([self.q(state, s, action, a, l) for l in L])
        softmax = (np.exp(-1.0 * X)/np.sum(np.exp(-1.0 * X)))
        for l in range(len(L)):
            self.potential[s][a][L[l]] += self.persistence * softmax[l]
        self.potential[s][a] = np.clip(self.potential[s][a], a_min = 0, a_max = 1)


    def update_kappa(self):
        """
            params:
                None

            returns:
                None

            description:
                For every state and action, update the potential for their current max
                level of autonomy and all adjacent levels in L.
                After, using the gated exploration policy determine if kappa(s,a) should
                be changed to one of the adjacent levels.

            notes:
                Currently we are assuming that L is a total ordering, so the only levels
                adjacant to kappa(s,a) are one above and one below.
        """
        for s, state in enumerate(self.DM.states):
            for a, action in enumerate(self.DM.actions):
                if action not in self.HM.flagged:
                    continue

                level = self.AM.kappa[state][action]
                if level == 0 or level == max(self.AM.L):
                    continue
                else:
                    L = [level-1, level, level+1]
                self.update_potential(state, s, action, a, L)

                for level_index in np.argsort(-1.0 * np.array([self.potential[s][a][l] for l in L])):
                    if np.random.uniform() <= self.potential[s][a][L[level_index]]:
                        if L[level_index] == 3 and len(state) > 2:
                            if ((state[3] == 'door-closed' and action == 'open'
                                and (self.DM.helper.get_state_feature_value(state,'doortype') == 'pull'
                                 or self.DM.helper.get_state_feature_value(state, 'doorsize') == 'large'
                                 or (self.DM.helper.get_state_feature_value(state, 'doorsize') == 'medium'
                                 and self.DM.helper.get_state_feature_value(state, 'region') == 'b2')))
                                or (action[0] == 'cross' and (state[3] == 'busy' or (state[3] == 'light'
                                and self.DM.helper.get_state_feature_value(state, 'visibility') == 'low')))):
                                self.potential[s][a][L[level_index]] = 0.0
                                break
                            elif self.HM.lambda_[state][action][2] < 0.95:
                                break

                        if L[level_index] == 0 and len(state) > 2:
                            if ((state[3] == 'door-closed' and action == 'open'
                                and self.DM.helper.get_state_feature_value(state, 'doortype') == 'push'
                                and (self.DM.helper.get_state_feature_value(state, 'doorsize') == 'small'
                                 or (self.DM.helper.get_state_feature_value(state, 'doorsize') == 'medium'
                                 and self.DM.helper.get_state_feature_value(state, 'region') != 'b2')))
                            or (state[3] == 'empty' or state[3] == 'light'
                                and (self.DM.helper.get_state_feature_value(state, 'visibility') == 'high')
                                and action == 'cross')):
                                self.potential[s][a][L[level_index]] = 0.0
                                break
                            elif self.HM.lambda_[state][action][1] > 0.25:
                                break

                        self.AM.kappa[state][action] = L[level_index]
                        self.potential[s][a][L[level_index]] = 0.0

                        if L[1] == 1 and L[level_index] == 2:
                            self.flags[s][a] = True
                        break


    def save_kappa(self):
        self.AM.save_kappa()


    def solve(self, solver='FVI'):
        """
            params:
                solver - The solver to be using. Defaults to FVI.

            returns:
                runtime - The time it took to solve for benchmarking purposes.

            description:
                This function runs a given solver, e.g. FVI, to produce
                    pi - The optimal policy for the agent.
                    state_map - A dictionary mapping states to state indices.
                    V - The value function in matrix form over states.
                    Q - The q-value funciton in matrix for over state action pairs.
        """
        start_time = time.time()
        mdp_info = FVI(self)
        end_time = time.time()

        self.pi = mdp_info['pi']
        self.state_map = mdp_info['state map']
        self.V = mdp_info['V']
        self.Q = mdp_info['Q']

        return (end_time - start_time)


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


    def remove_transition(self, state, action):
        """
            params:
                state - The state we are removing the transition for.
                action - The action we are removing the transition for.

            returns:
                None

            description:
                Removes the transition for some (s,a). In particular this means
                putting all transition probability on T(s,a) into T[s][a][s].
                I.e. if the agent tries to take this action in this state again
                they will remain in the same state.
        """
        s = self.states.index(state)

        for i in np.arange(1, len(self.AM.L)):
            a = self.actions.index((action[0], i))
            self.transitions[s][a] *= 0.0
            self.transitions[s][a][s] = 1.0


    def check_level_optimality(self):
        """
            params: None

            returns: The percent of states for which the policy is level-optimal.
        """
        total, correct = 0., 0.
        for s, state in enumerate(self.states):
            if len(state[0]) < 3 or 'open' in state[0][3]:
                continue
            total += 1
            action = self.pi[s]
            correct = correct + self.DM.helper.level_optimal(state, action)

        return correct/total