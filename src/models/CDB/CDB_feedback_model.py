import os, sys, time, random, pickle
import numpy as np
import itertools as it

from IPython import embed

current_file_path = os.path.dirname(os.path.realpath(__file__))
sys.path.append(os.path.join(current_file_path, '..'))

PARAM_PATH = os.path.join('..', 'data', 'model parameters')

class FeedbackModel():
    def __init__(self, DM, AM, Sigma, flagged):
        self.DM = DM
        self.AM = AM
        self.Sigma = Sigma
        self.flagged = flagged
        self.lambda_ = self.generate_lambda()
        self.T_H = self.generate_T_H()

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
                for level in [1,2]:
                    p = self.DM.helper.predict(state, action, level)
                    lambda_[state][action][level] = p
        return lambda_

    def predict_feedback_probability(self, state, action, sigma):
        p = -1.0
        if sigma == '+':
            p = self.lambda_[state][action][1]
        elif sigma == '-':
            p = 1 - self.lambda_[state][action][1]
        elif sigma == None:
            p = self.lambda_[stat][action][2]
        elif sigma == '/':
            p = 1 - self.lambda_[state][action][2]
        return p if 0. <= p <= 1. else 0.

    def tau(self, state, state_level, action, action_level, statePrime):
        """
            This function is going to encode the 'feedback-dynamics', so-called,
            for any set of feedback. In general, this will need to be coded by hand.
            In this case for the standard CAS model with feedback signals.
        """
        if action_level not in self.flagged or action_level == 3:
            return 1.

        tau_ = 0.0
        if action[1] == 1:
            tau_ = (self.predict_feedback_probability(state, action, '+')
                 + self.predict_feedback_probability(state, action, '-') * (state == statePrime))
        elif action[1] == 2:    # Initial assumption that when human overrides, they simply stop the agent.
            tau_ = (self.predict_feedback_probability(state, action, None)
                 + self.predict_feedback_probability(state, action, '/') * (state == statePrime))

        return tau_

    def generate_T_H(self):
        """
            This function is supposed to represent the transition dynamics of the
            human when they are in control. In principle, this is a function that
            would probably learned through experience.
            Short of learning it, or as a baseline, we would like to say that this
            function optimistically assumes that the human *succeeds* at whatever
            the action that the agent is trying to perform is. Hence we use a MLO
            determinization as the function's baseline.
        """
        row_maxes = self.DM.transitions.max(axis=2, keepdims=True)
        return np.where(self.DM.transitions == row_maxes, 1., 0.)

    def rho(self, state, action):
        return len(self.AM.L) - action[1]

    def find_candidates(delta=0.1, thresh=100):
    """
        params:
            delta  - Defines the probability threshold needed for (s,a) to not be a candidate.
            thresh - Defines the datacount threshold needed for (s,a) to be a candidate
                     if it does not meet the probability threshold.
    """
    candidates = []
    for state in self.lambda_.keys():
        for action in self.lambda_[state].keys():
            candidate = True
            for sigma in self.Sigma:
                if self.predict_feedback_probability(state, action, sigma) > 1 - delta:
                    candidate = False
            if candidate:
                candidates.append((state, action))

    return candidates

    def get_most_likely_discriminators(D, candidate, k):
    """
        params:
            D - The data matrix being used to produce the discriminators.
            candidate - A candidate (s,a) for feature augmentation.
            k - The number of features to be returned.

        returns:
            discriminators - A list of features.
    """
    unused_features = get_unused_features(D, state)             # TODO: Implement this function
    C = generate_correlation_matrix(unused_features, D)         # TODO: Implement this function
    D = generate_discrimination_matrix(unused_features, C)      # TODO: Implement this function

    return unused_features[np.argmax(D, k)]

    def test_discriminators(D_train, D_test, discriminators):
    """
        params:
            D_train        - The data matrix that is going to be used to train the classifiers.
                             This should be the same or smaller than what was used to produce them.
            D_test         - The data matrix that is going to be used to test the classifiers.
                             No data in this should be used to determine the discriminators.
            discriminators - The discriminators to be testing, where each is an *unused feature*
                             available to the agent in the full data matrix.

        returns:
            d*             - The discriminator which produced the classifier with the highest
                             performance on the test data.
    """
    lambdas = [build_lambda(discriminator, D_train) for discriminator in discriminators]    #TODO: Implement this function

    accuracies = [test_lambda(lambda_, D_test) for lambda_ in lambdas]                      #TODO: Implement this function

    return discriminators[np.argmax(accuracies)]