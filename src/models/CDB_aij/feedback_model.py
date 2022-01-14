import os, sys, time, random, pickle, json
import numpy as np
import pandas as pd
import itertools as it

from IPython import embed
from sklearn.metrics import matthews_corrcoef, adjusted_mutual_info_score

current_file_path = os.path.dirname(os.path.realpath(__file__))
sys.path.append(os.path.join(current_file_path, '..'))

from scripts.utils import build_gam

DOMAIN_PATH = os.path.join(current_file_path, '..', '..', '..', 'domains', 'CDB_aij')
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

    def find_candidates(self, delta=0.05, thresh=60):
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

    def get_most_likely_discriminator(self, candidate, k, scoring_function='mRMR'):
        """y
            params:
                D - The data matrix being used to produce the discriminators.
                candidate - A candidate (s,a) for feature augmentation.
                k - The number of features to be returned.

            returns:
                discriminators - A list of features.
        """
        print("Finding most likely discriminator...")
        state, action, level = candidate

        # First, for a given candidate, procure its unused features
        unused_features = self.get_unused_features(state, action)

        if len(unused_features) == 0:
            return None
            
        # Second, compute the correlation matrix over each unused feature
        D = pd.read_csv(os.path.join(FEEDBACK_PATH, action + "_full.data"))
        
        if scoring_function == 'mRMR':
            _disc = dict()
            for i in range(len(unused_features)):
                f1 = unused_features[i]
                _disc[f1] = self.mRMR(D, f1)
                # for j in range(i+1, len(unused_features)):
                #     f2 = unused_features[j]
                #     _disc[(f1 + ',' + f2)] = self.mRMR(D, [f1, f2])

        if scoring_function == 'd_index':
            _disc = d_index(D, unused_features)

        discriminators = np.array(list(_disc.keys()))[np.argpartition(np.array(list(_disc.values())), -min(k, len(list(_disc.values()))))[-min(k, len(list(_disc.values()))):]]
        
        D_train = D.sample(frac=0.75)

        D_test = D.drop(D_train.index)

        return self.test_discriminators(D_train, D_test, D.columns.drop(np.append(unused_features, 'feedback')), discriminators)


    def d_index(self, df, features):
        X = pd.get_dummies( df[np.append(features, 'feedback')] )
        cols = X.columns.values
        for i in range(len(cols)):
            f1 = cols[i]
            for j in range(i+1, len(cols)):
                f2 = cols[j]
                try:
                    if f1[:f1.index('_')] != f2[:f2.index('_')] and 'feedback' not in f1 and 'feedback' not in f2:
                        f3 = f1 + ',' + f2
                        X[f3] = X[f1] * X[f2]
                except:
                    if f1 != f2 and 'feedback' not in f1 and 'feedback' not in f2:
                        f3 = f1 + ',' + f2
                        X[f3] = X[f1] * X[f2] 

        _corr = X.corr()
        _corr = _corr[[c for c in _corr.columns.values if 'feedback' in c]]
        _corr = _corr.drop([c for c in _corr.axes[0].values if 'feedback' in c], axis = 0)
        _corr = _corr.fillna(0)

        # print("Built correlation matrix...")

        # Third, build the discrimantor matrix for each feature or pair
        # of features present in the correlation matrix. Right now we are simply
        # doing this by taking the max over all feature-values, and then summing
        # over each feature value's max correlation with the feedback for the
        # relevant Feature.
        _disc = dict()
        for i in range(len(unused_features)):
            f1 = unused_features[i]
            _disc[f1] = 0
            for j in range(i+1, len(unused_features)):
                f2 = unused_features[j]
                _disc[(f1 + ',' + f2)] = 0
        for row_name in _corr.axes[0].values:
            if ',' in row_name:
                feature_values = row_name.split(',')
                feature_value_1, feature_value_2 = feature_values[0], feature_values[1]
                if '_' in feature_value_1:
                    feature_1 = feature_value_1[:feature_value_1.index('_')]
                else:
                    feature_1 = feature_value_1
                if '_' in feature_value_2:
                    feature_2 = feature_value_2[:feature_value_2.index('_')]
                else:
                    feature_2 = feature_value_2
                pairwise_feature = feature_1 + ',' + feature_2
                _disc[pairwise_feature] += np.max(_corr.loc[row_name])
            else:
                if '_' in row_name:
                    f = row_name[:row_name.index('_')]
                    _disc[f] += np.max(_corr.loc[row_name])
                else:
                    _disc[row_name] += np.max(_corr.loc[row_name])

        # print("Built discrimination matrix...")

        indices = list(_corr.index.values)
        for f in _disc.keys():
            count = 0
            if ',' in f:
                F = f.split(',')
                f1, f2 = F[0], F[1]
                count = len([v for v in indices if (f1 in v and f2 in v)])
            else:
                count = len([v for v in indices if (f in v and ',' not in v)])
            _disc[f] /= count

        return _disc


    def mRMR(self, df, features):
        X = pd.get_dummies(df[features])
        try:
            y = (np.array(list(df['feedback'])) == 'yes').astype(int)
        except Exception:
            embed()
        relevance = 0
        for f in X.columns.values:
            x = np.array(list(X[f]))
            relevance += adjusted_mutual_info_score(x, y)
        relevance /= len(X.columns.values)

        repetition = 0
        for i in range(len(X.columns.values)):
            for j in range(i+1, len(X.columns.values)):
                x_i = np.array(list(X[X.columns.values[i]]))
                x_j = np.array(list(X[X.columns.values[j]]))
                repetition += adjusted_mutual_info_score(x_i, x_j)
        repetition /= (len(X.columns.values) ** 2)

        return relevance - repetition
 
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
        print("Testing discriminators....")
        lambdas = [(self.build_lambda(D_train, used_features, discriminator), discriminator) for discriminator in discriminators]
        scores = [self.test_lambda(lambda_, lambda_map, D_test, used_features, discriminator) for (lambda_, lambda_map), discriminator in lambdas]

        print("Checkpoint 1.5")

        best_score = np.max(scores)
        best_discrims = []
        for i in range(len(discriminators)):
            if scores[i] == best_score:
                best_discrims.append(discriminators[i])
        d = best_discrims[0]
        for discriminator in best_discrims:
            if ',' not in discriminator:
                d = discriminator
                break

        print("Checking discriminator " + str(d) + "...")
        print("Checkpoint 2")

        if 'traffic' in D_train.columns:
            curr_score = self.test_lambda(self.DM.helper.cross_GAM, self.DM.helper.cross_GAM_map, D_test, used_features, discriminator = None)
            if np.max(scores) < curr_score + 0.05 or np.max(scores) < 0.5 or curr_score == -1.0:
                return None
        else:
            curr_score = self.test_lambda(self.DM.helper.open_GAM, self.DM.helper.open_GAM_map, D_test, used_features, discriminator = None) 
            if np.max(scores) < curr_score + 0.05 or np.max(scores) < 0.5 or curr_score == -1.0:
                return None

        print("Checkpoint 3")
        return d

    def build_lambda(self, D_train, used_features, discriminator):
        """
            params:
                D_train         - The data matrix that is going to be used to train the classifiers.
                used_features   - The set of features that are currently being used by the system
                discriminator   - The feauture(s) we are testing adding to the feedback profile.

            returns:
                lambda_         - The GAM produced when training with the new discriminator added on D_train.
                lambda_map      - The conversion map for the gam.
        """
        print("Building lambda for discriminator {}".format(discriminator))
        if ',' in discriminator:
            discriminator_values = discriminator.split(',')
            train = D_train[np.append(used_features, discriminator_values + ['feedback'])]
        else:
            train = D_train[np.append(used_features, [discriminator, 'feedback'])]

        lambda_, lambda_map = build_gam(train, fast=False)

        return (lambda_, lambda_map)

    def test_lambda(self, lambda_, lambda_map, D_test, used_features, discriminator):
        """
            params:
                lambda_         - The feedback profile we are testing.
                lambda_map      - The conversion map for the feedback profile.
                D_test          - The test split of our dataset.
                used_features   - The set of features that are currently being used by the system.
                discriminator   - The feature(s) we are testing adding to the feedback profile.

            return:
                The f1 score of the new feedback profile. 
        """
        print("Testing discriminator {}".format(discriminator))
        if lambda_ is None:
            return -1.

        if discriminator == None:
            X = D_test[used_features]
        else:
            if ',' in discriminator:
                discriminator_values = discriminator.split(',')
                X = D_test[np.append(used_features, discriminator_values)]
            else:
                X = D_test[np.append(used_features, discriminator)]
        y = D_test['feedback'] == 'yes'
        y_true = np.array(y, int)

        X_ = []
        for x in X.values:
            x_arr = []
            for f in x:
                if type(f) == float:
                    x_arr.append(f)
                else:
                    x_arr.append(lambda_map[f])
            X_.append(x_arr)

        X_ = np.array(X_)

        predictions = lambda_.predict(X_)
        y_pred = predictions > 0.5

        print("Checkpoint 1")
        # if matthews_corrcoef(y_true, y_pred) > 0.5:
        #     embed()

        return matthews_corrcoef(y_true, y_pred)