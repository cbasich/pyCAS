import os, sys, time, random, pickle, json
import numpy as np
import pandas as pd
import itertools as it

from IPython import embed

current_file_path = os.path.dirname(os.path.realpath(__file__))
sys.path.append(os.path.join(current_file_path, '..'))

from scripts.utils import build_gam

DOMAIN_PATH = os.path.join(current_file_path, '..', '..', '..', 'domains', 'CDB')
FEEDBACK_PATH = os.path.join(DOMAIN_PATH, 'feedback')
PARAM_PATH = os.path.join(DOMAIN_PATH, 'params')
MAP_PATH = os.path.join(DOMAIN_PATH, 'maps')

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
            if len(state) == 2:
                continue
            if not state in lambda_.keys():
                lambda_[state] = {}
            for action in self.flagged:
                if not action in lambda_[state].keys():
                    lambda_[state][action] = {}
                for level in [1,2]:
                    try:
                        p = self.DM.helper.predict(state, action, level)
                    except Exception:
                        p = 0.5
                    lambda_[state][action][level] = p
        return lambda_

    def predict_feedback_probability(self, state, action, sigma):
        p = -1.0
        if sigma == '+':
            p = self.lambda_[state][action][1]
        elif sigma == '-':
            p = 1 - self.lambda_[state][action][1]
        elif sigma == None:
            p = self.lambda_[state][action][2]
        elif sigma == '/':
            p = 1 - self.lambda_[state][action][2]
        return p if 0. <= p <= 1. else 0.

    def tau(self, s, state, state_level, a, action, action_level, sp, statePrime):
        """
            This function is going to encode the 'feedback-dynamics', so-called,
            for any set of feedback. In general, this will need to be coded by hand.
            In this case for the standard CAS model with feedback signals.
        """
        p = self.DM.transitions[s][a][sp]

        if len(state) == 2 or action not in self.flagged or action_level == 3:
            return p

        tau_ = 0.0
        if action_level == 1:
            tau_ = (self.predict_feedback_probability(state, action, '+') * p
                 + self.predict_feedback_probability(state, action, '-') * (state == statePrime))
        elif action_level == 2:    # Initial assumption that when human overrides, they simply stop the agent.
            tau_ = (self.predict_feedback_probability(state, action, None) * p
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
        if action[1] == 0:
            return 10
        elif action[1] == 1:
            return 2
        elif action[1] == 2:
            return 1
        else:
            return 0.

    def get_unused_features(self, state, action):
        """
            params:
                state    - The state that we are getting unused features for.
                action   - The action in question; used for loading the dataframe.

            returns:
                features - The list of features that are currently not being used
                           in the factored representation of the state.
        """
        data_file = os.path.join(FEEDBACK_PATH, action+'.data')
        df = pd.read_csv(data_file)

        data_file_full = os.path.join(FEEDBACK_PATH, action+'_full.data')
        df_full = pd.read_csv(data_file_full)

        # Get the column names which are the features currently being used.
        used_features = df.columns

        # Get the complete list of features available.
        full_features = df_full.columns

        # Get the features in full_features not in used_features
        unused_features = full_features.drop(used_features)
        return unused_features

    def find_candidates(self, delta=0.1, thresh=30):
        """
        params:
            delta  - Defines the probability threshold needed for (s,a) to not be a candidate.
            thresh - Defines the datacount threshold needed for (s,a) to be a candidate
                     if it does not meet the probability threshold.
        """
        init_state_candidate_count = 0

        # Load the relevant data files
        cross_path = os.path.join(FEEDBACK_PATH, 'cross.data')
        open_path = os.path.join(FEEDBACK_PATH, 'open.data')
        cross_df = pd.read_csv(cross_path)
        open_df = pd.read_csv(open_path)

        with open(os.path.join(MAP_PATH, 'map_info.json')) as F:
            map_info = json.load(F)

        with open(os.path.join(PARAM_PATH, 'used_features.txt')) as F:
            used_features = F.readline().split(',')

        candidates = []
        for state in self.lambda_.keys():
            dic = map_info[str((state[0], state[1]))]
            if dic["obstacle"] == "crosswalk":
                dic["obstacle"] = state[3]
            for action in self.lambda_[state].keys():
                for level in self.lambda_[state][action].keys():

                    # Get the count for thist (s, a, l) tuple in the relevant datafile.
                    if action == 'open':
                        count = np.sum(pd.DataFrame([open_df[k] == v for k,v in dic.items() if k in open_df.columns.values]).all())
                    elif action == 'cross':
                        count = np.sum(pd.DataFrame([cross_df[k] == v for k,v in dic.items() if k in cross_df.columns.values]).all())

                    # If we have not seen this (s, a, l) a sufficient number of times, skip it.
                    if count < thresh: 
                        continue

                    candidate = True

                    # Determine if there is *any* feedback signal that we can predict with probability
                    # at least 1 - delta for some given delta. If there isn't, then this (s, a, l) is
                    # considered to be a candidate. 
                    for sigma in self.Sigma:
                        if self.predict_feedback_probability(state, action, sigma) > 1 - delta:
                            candidate = False

                    if candidate:
                        candidates.append((state, action, level))
                        if len(state) == 4:
                            init_state_candidate_count += 1

        return candidates, init_state_candidate_count

    def get_most_likely_discriminator(self, candidate, k):
        """y
            params:
                D - The data matrix being used to produce the discriminators.
                candidate - A candidate (s,a) for feature augmentation.
                k - The number of features to be returned.

            returns:
                discriminators - A list of features.
        """
        state, action, level = candidate

        # First, for a given candidate, procure its unused features
        unused_features = self.get_unused_features(state, action)

        if len(unused_features) == 0:
            return None
            
        # Second, compute the correlation matrix over each unused feature
        # and the feedback.
        # TODO: Include all *pairs* of features as rows in the matrix.
        # TODO: There is an issue right now with features that take on <= 2
        #       integral values (i.e. level) in the presence of objects. The issue
        #       is that during the one hot encoding, each value (i.e. level 1, level 2)
        #       is not assigned a separate row. So there is a correlation only for 'feature'
        #       whereas the other features have a correlation for each 'feature-value'.
        #       This may present problems below when calculating the discriminator matrix.
        #       The real todo is check if that is the case or not.
        D = pd.read_csv(os.path.join(FEEDBACK_PATH, action + "_full.data"))
        _corr = pd.get_dummies( D[np.append(unused_features, 'feedback')] ).corr()
        _corr = _corr[[c for c in _corr.columns.values if 'feedback' in c]]
        _corr = _corr.drop([c for c in _corr.axes[0].values if 'feedback' in c], axis = 0)


        # Third, build the discrimantor matrix for each feature or pair
        # of features present in the correlation matrix. Right now we are simply
        # doing this by taking the max over all feature-values, and then summing
        # over each feature value's max correlation with the feedback for the
        # relevant Feature.
        _disc = {f: 0 for f in unused_features}
        for row_name in _corr.axes[0].values:
            try:
                f = row_name[:row_name.index('_')]
                _disc[f] += np.max(_corr.loc[row_name])
            except:
                _disc[row_name] += np.max(_corr.loc[row_name])

        indices = list(_corr.index.values)
        for f in _disc.keys():
            _disc[f] /= len([v for v in indices if f in v])
        discriminators = unused_features[np.argpartition(np.array(list(_disc.values())), -k)[-k:]]

        D_train = D.sample(frac=0.75)
        D_test = D.drop(D_train.index)

        return self.test_discriminators(D_train, D_test, D.columns.drop(np.append(unused_features, 'feedback')), discriminators)
 
    def test_discriminators(self, D_train, D_test, used_features, discriminators):
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
        lambdas = [(self.build_lambda(D_train, used_features, discriminator), discriminator) for discriminator in discriminators]
        accuracies = [self.test_lambda(lambda_, lambda_map, D_test, used_features, discriminator) for (lambda_, lambda_map), discriminator in lambdas]
        d = discriminators[np.argmax(accuracies)]

        print("Checking discriminator " + str(d) + "...")

        if 'door' in D_train['obstacle'].values:
            curr_acc = self.test_lambda(self.DM.helper.open_GAM, self.DM.helper.open_GAM_map, D_test, used_features, discriminator = None)
            if np.max(accuracies) - curr_acc < 0.1:
                return None
        else:
            curr_acc = self.test_lambda(self.DM.helper.cross_GAM, self.DM.helper.cross_GAM_map, D_test, used_features, discriminator = None)
            if np.max(accuracies) - curr_acc < 0.1:
                return None

        return d

    def build_lambda(self, D_train, used_features, discriminator):
        train = D_train[np.append(used_features, [discriminator, 'feedback'])]

        gam, gam_map = build_gam(train)

        return (gam, gam_map)

    def test_lambda(self, lambda_, lambda_map, D_test, used_features, discriminator):
        if discriminator == None:
            X = D_test[used_features]
        else:
            X = D_test[np.append(used_features, discriminator)]
        y = D_test['feedback'] == 'yes'

        X_ = np.array([[lambda_map[f] for f in x] for x in X.values])

        predictions = lambda_.predict(X_) > 0.5

        accuracy = np.sum( predictions == y ) / len(y)

        return accuracy