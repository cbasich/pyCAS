import os, sys, time, random, pickle, json
import numpy as np
import pandas as pd
import itertools as it

from IPython import embed
from sklearn.preprocessing import OneHotEncoder
from sklearn.metrics import matthews_corrcoef, adjusted_mutual_info_score

current_file_path = os.path.dirname(os.path.realpath(__file__))
sys.path.append(os.path.join(current_file_path, '..'))

from scripts.utils import _train_model, _train_model_naive, _train_model_soft_labeling, _train_model_multi_task, _train_model_multi_source

DOMAIN_PATH = os.path.join(current_file_path, '..', '..', '..', 'domains', 'CDB_icra')
FEEDBACK_PATH = os.path.join(DOMAIN_PATH, 'feedback')
PARAM_PATH = os.path.join(DOMAIN_PATH, 'params')
MAP_PATH = os.path.join(DOMAIN_PATH, 'maps')

class FeedbackModel():
    def __init__(self, DM, AM, Sigma, flagged_actions, training_method):
        self.DM = DM
        self.AM = AM
        self.Sigma = Sigma
        self.flagged_actions = flagged_actions
        self.open_features = ['region', 'doorsize', 'doortype']
        self.cross_features= ['region', 'traffic', 'visibility', 'timeofday']
        self.open_data, self.cross_data = self._initialize_data('open'), self._initialize_data('cross')
        # self.open_data_recent, self.cross_data_recent = np.array([]), np.array([])
        self.open_enc = OneHotEncoder(handle_unknown='ignore').fit(self.open_data[:,:-1])
        self.cross_enc = OneHotEncoder(handle_unknown='ignore').fit(self.cross_data[:,:-1])
        self.open_classifier, self.cross_classifier, self.lambda_, self.T_H= None, None, None, None
        self.training_method = training_method


    def _initialize_data(self, action):
        if action == 'open':
            return np.array([list(item) for item in it.product([1, 2], ['b1', 'b2', 'b3'], 
                ['small', 'medium', 'large'], ['push', 'pull'], ['yes', 'no'])])
        elif action == 'cross':
            return np.array([list(item) for item in it.product([1, 2], ['r1', 'r2'], ['empty', 'light', 'busy'], 
                ['low', 'high'], ['morning', 'midday', 'afternoon'], ['yes', 'no'])])
        else:
            return None


    def _update_data(self, state, action, feedback, flagged):
        if action[0] == 'open':
            entry = [str(action[1])] + [self.DM.helper.get_state_feature_value(state[0], f)
                        for f in self.open_features if self.DM.helper.get_state_feature_value(state[0], f) != None] + [feedback]
            self.open_data = np.append(self.open_data, [entry], axis = 0)
                # self.open_data_recent = np.append(self.open_data_recent, [entry], axis = 0)
            if (action[1] == 1 and flagged == False):
                entry_copy = entry
                entry_copy[0] = 2
                self.open_data = np.append(self.open_data, [entry_copy], axis = 0)
                # self.open_data_recent = np.append(self.open_data_recent, [entry_copy], axis = 0)
        elif action[0] == 'cross':
            entry = [str(action[1])] + [self.DM.helper.get_state_feature_value(state[0], f)
                        for f in self.cross_features if self.DM.helper.get_state_feature_value(state[0], f) != None] + [feedback]
            self.cross_data = np.append(self.cross_data, [entry], axis = 0)
            # self.cross_data_recent = np.append(self.cross_data_recent, [entry], axis = 0)
            if (action[1] == 1 and flagged == False):
                entry_copy = entry
                entry_copy[0] = 2
                self.cross_data = np.append(self.cross_data, [entry_copy], axis = 0)
                # self.cross_data_recent = np.append(self.cross_data_recent, [entry_copy], axis = 0)


    def get_classifier(self, action):
        if action == 'open':
            return self.open_classifier
        elif action == 'cross':
            return self.cross_classifier


    def train(self, agent_id, action, training_method='none'):
        assert (action == 'open' or action == 'cross')
        classifier = None

        if action == 'open':
            X = self.open_enc.transform(self.open_data[:,:-1]).toarray()
            y = self.open_data[:,-1:] == 'yes'
        elif action == 'cross':
            X = self.cross_enc.transform(self.cross_data[:,:-1]).toarray()
            y = self.cross_data[:,-1:] == 'yes'
        y = y.reshape(-1,)

        if training_method == 'none':
            classifier = _train_model(X, y)
        elif training_method == 'naive':
            classifier = _train_model_naive(agent_id, X, y, action)
        elif training_method == 'multi_task':
            classifier = _train_model_multi_task(agent_id, X, y, action)
        elif training_method == 'multi_source':
            classifier = _train_model_multi_source(agent_id, X, y, action)
        elif training_method == 'soft_labeling':
            if action == 'open':
                classifier = _train_model_soft_labeling(agent_id, X, y, action, self.open_classifier)
            elif action == 'cross':
                classifier = _train_model_soft_labeling(agent_id, X, y, action, self.cross_classifier)
        
        if action == 'open':
            self.open_classifier = classifier
        elif action == 'cross':
            self.cross_classifier = classifier


    def generate_lambda(self):
        """
            This function encodes the feedback profile, and consequently
            must be hand coded for the specific model at hand. In this case
            it is for the standard CAS model with 4 feedback signals.

            Note: In this specific model, feedback is only received in l1 & l2
        """
        lambda_ = {}
        for state in self.DM.states:
            if len(state) == 2:
                continue
            if not state in lambda_.keys():
                lambda_[state] = {}
            for action in self.flagged_actions:
                if not action in lambda_[state].keys():
                    lambda_[state][action] = {}
                for level in [1,2]:
                    # try:
                    features = [level] + self.DM.helper.extract_state_features(state)
                    p = self._predict(features, action)
                    # except Exception:
                    #     p = 0.5
                    lambda_[state][action][level] = p
        self.lambda_ = lambda_


    def update_lambda(self):
        """
            This function updates lambda to save time rather than regenerating every call.
        """
        for state in self.lambda_.keys():
            for action in self.lambda_[state].keys():
                for level in self.lambda_[state][action].keys():
                    features = [level] + self.DM.helper.extract_state_features(state)
                    self.lambda_[state][action][level] = self._predict(features, action)


    def _predict(self, features, action):
        """
            params:
                state  - The state we are predicting the feedback probability for.
                action - The action we are predicting the feedback probability for.
                level  - The level we are predicting the feedback probability for.

            returns:
                The probability of a 'positive' response, which, for level 2,
                means the probability of *no* override, and for level 1, means the probability 
                of approval. We return -1.0 if this is an input that we don't have data for.
        """
        try:
            if action == 'open':
                x = self.open_enc.transform([features])
                return self.open_classifier.predict(x.toarray())[0]
            elif action == 'cross':
                x = self.cross_enc.transform([features])
                return self.cross_classifier.predict(x.toarray())[0]
        except:
            return 0.5


    def predict(self, state, action, sigma):
        p = -1.0
        if sigma == '+':
            p = self.lambda_[state][action][1]
        elif sigma == '-':
            p = 1 - self.lambda_[state][action][1]
        elif sigma == None:
            p = self.lambda_[state][action][2]
        elif sigma == '/':
            p = 1 - self.lambda_[state][action][2]
        # if self.training_method == 'soft_labeling':
        #     p += 0.5000
        return p if 0. <= p <= 1. else 0.


    def tau(self, s, state, state_level, a, action, action_level, sp, statePrime):
        """
            This function is going to encode the 'feedback-dynamics', so-called,
            for any set of feedback. In general, this will need to be coded by hand.
            In this case for the standard CAS model with feedback signals.
        """
        p = self.DM.transitions[s][a][sp]

        if len(state) == 2 or action not in self.flagged_actions or action_level == 3:
            return p

        tau_ = 0.0
        if action_level == 1:
            tau_ = (self.predict(state, action, '+') * p
                 + self.predict(state, action, '-') * (state == statePrime))
        elif action_level == 2:    # Initial assumption that when human overrides, they simply stop the agent.
            tau_ = (self.predict(state, action, None) * p
                 + self.predict(state, action, '/') * (state == statePrime))

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
        self.T_H = np.where(self.DM.transitions == row_maxes, 1., 0.)


    def rho(self, state, action):
        if action[1] == 0:
            return 10
        elif action[1] == 1:
            return 2
        elif action[1] == 2:
            return 1
        else:
            return 0.
