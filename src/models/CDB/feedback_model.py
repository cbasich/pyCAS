import os, sys, time, random, pickle
import numpy as np
import itertools as it

current_file_path = os.path.dirname(os.path.realpath(__file__))
sys.path.append(os.path.join(current_file_path, '..'))

from utils.utils import predict

PARAM_PATH = os.path.join('..', 'data', 'model parameters')

class HM():
    def __init__(self, DM, AM, Sigma, flagged):
        self.DM = DM
        self.AM = AM
        self.Sigma = Sigma
        self.flagged = flagged
        self.lambda_ = self.generate_lambda()

    def generate_lambda(self):
        """
            This function encodes the feedback profile, and consequently
            must be hand coded for the specific model at hand. In this case
            it is for the standard CAS model with 4 feedback signals.

            Note: In this specific model, feedback is only received in l1 & l2
        """
        lambda_ = {}
        for state in self.DM.states:        # Assumption that level component doesnt impact lambda
            if not state in lambda_.keys():
                lambda_[state] = {}
            for action in self.flagged:
                if not action in lambda_[state].keys():
                    lambda_[state][action] = {}
                for level in [1.2]:
                    p = predict('gridworld', action, [level, self.DM.regions[state], state[3]])
                    if p != -1.0:
                        lambda_[state][action][level] = p
        return lambda_

    def predict_feedback(self, state, action, sigma):
        

    def tau(self, s, state, a, action, sp, statePrime):
        """
            This function is going to encode the 'feedback-dynamics', so-called,
            for any set of feedback. In general, this will need to be coded by hand.
            In this case for the standard CAS model with feedback signals.
        """
        tau = 0.0
        if action[1] == 1:
            tau = (self.predict_feedback(state, action, '+')
                 + self.predict_feedback(state, action, '-') * (state == statePrime))
        elif action[1] == 2:
            tau = (self.predict_feedback(state, action, None)
                 + self.predict_feedback(state, action, '/') * self.T_H(state, action, statePrime))

        return tau

    def T_H(self, state, action, statePrime):
        """
            This function is supposed to represent the transition dynamics of the
            human when they are in control. In principle, this is a function that
            would probably learned through experience.
            Short of learning it, or as a baseline, we would like to say that this
            function optimistically assumes that the human *succeeds* at whatever
            the action that the agent is trying to perform is. Hence we use a MLO
            determinization as the function's baseline.
        """
        s = self.DM.states.index(state)
        a = self.DM.actions.index(action)
        sp = self.DM.states.index(statePrime)

        return 1. if sp == np.argmax(self.DM.transitions[s][a]) else 0.


    def rho(self, l):
        return len(self.AM.L) - l