import os, sys, time, random, pickle

import numpy as np
import pandas as pd

from IPython import embed

current_file_path = os.path.dirname(os.path.realpath(__file__))
sys.path.append(os.path.join(current_file_path, '..', '..'))

from scripts.utils import build_gam

DOMAIN_PATH = os.path.join(current_file_path, '..', '..', '..', 'domains', 'CDB_icra')
FEEDBACK_PATH = os.path.join(DOMAIN_PATH, 'feedback')
PARAM_PATH = os.path.join(DOMAIN_PATH, 'params')


class Helper():
    def __init__(self, DM):
        self.DM = DM
        self.map_info = self.DM.map_info
        
    def level_optimal(self, state, action):
        """
            params:
                state  - The state that we are checking the level optimality for.
                action - The action that we are checking the level optimality for.

            returns:
                True if the action is being taken at the optimal level in state. 
        """
        state = state[0]        # This is a HACK. timeofdayO: Remove when level matters for states
        level = action[1]

        if len(state) < 3:
            return level == 3

        if ((state[3] == 'door-closed' and action[0] == 'open'
            and (self.get_state_feature_value(state, 'doortype') == 'pull'
            or (self.get_state_feature_value(state, 'doorsize') == 'large')
            or (self.get_state_feature_value(state, 'doorsize') == 'medium'
            and self.get_state_feature_value(state, 'region') == 'b2')))
        or (action[0] == 'cross' and (state[3] == 'busy' or (state[3] == 'light'
            and (self.get_state_feature_value(state, 'visibility') == 'low')
            or self.DM.timeofday in self.get_state_feature_value(state, 'pedestrians'))))):
            return level == 0

        return level == 3
        
    def extract_state_features(self, state):
        """
            params:
                state - The state that we are extracting features for.

            returns:
                f - The features of "state"

            description:
                Right now this function is returning the region and all state tuple elements that occur after index 2.
                This is because indices 0 and 1, the location, are captured by the region (which we must get manually)
                and if there is an index 2, it is always the heading of the agent which is not needed.

                As a result - any tuple indices past 2 are assumed to each have a unique feature. If that changes this
                function will need to be updated.

        """
        if str((state[0], state[1])) not in map_info.keys():
            return None
        else:
            if map_info[str((state[0], state[1]))]['obstacle'] == 'door':
                classes = ['region', 'doorsize', 'doortype']
            elif map_info[str((state[0], state[1]))]['obstacle'] == 'crosswalk':
                classes = ['region', 'visibility', 'timeofday', 'traffic']

        features = [self.get_state_feature_value(state, feature) for feature 
            in classes if self.get_state_feature_value(state, feature) != None]
        if 'crosswalk' in features:
            features[features.index('crosswalk')] = state[3]
        return features

    def predict(self, state, action, level):
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
        if len(state) == 2 or 'open' in state[3]:
            return 1.

        features = [level] + self.extract_state_features(state)

        if action == 'open' and 'door' in state[3]:
            return self.open_GAM.predict_mu([[self.open_GAM_map[f] for f in features]])
        elif action == 'cross' and 'door' not in state[3]:
            return self.cross_GAM.predict_mu([[self.cross_GAM_map[f] for f in features]])
        else:
            return -1.

    def get_state_feature_value(self, state, feature):
        """
            params:
                state - The state we are looking up the visibility condition for.
                feature - The feature *set* (i.e. 'doortype', 'visibility') of the state.

            returns:
                The feature *value* (i.e. 'heavy', 'high') of the state, or None if the state
                does not contain this feature.
        """
        if feature == 'timeofday':
            return self.DM.timeofday
        elif feature == 'traffic' and self.map_info[str((state[0], state[1]))]['obstacle'] == 'crosswalk':
            return state[3]
        try:
            return self.map_info[str((state[0], state[1]))][feature]
        except Exception:
            return None

    def add_feature(self, feature, candidate):
        """
            params:
                feature - The feature that we are adding to our model.
                candidate - The state action pair that we are augmenting with feature.

            returns:
                None

            description:
                After doing all of the work to determine the best feature to add
                this function is responsible for actually updating the datafile
                to appropriately contain a column for the new feature. Note that this
                does not add anything to the state representation, it only adds the feature
                to the data representation used by the classifier. 

        """
        state, action, _ = candidate

        df = pd.read_csv( open(os.path.join(FEEDBACK_PATH, action+'.data')))
        df_full = pd.read_csv( open(os.path.join(FEEDBACK_PATH, action+'_full.data')))
        # embed()
        if ',' in feature:
            features = feature.split(',')
            X = df_full[[f for f in list(df.columns.values) if f != 'feedback'] + features]
        else:
            X = df_full[[f for f in list(df.columns.values) if f != 'feedback'] + [feature]]
        X['feedback'] = df_full['feedback']

        X.to_csv(os.path.join(FEEDBACK_PATH, action+'.data'),index=False)