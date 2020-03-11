import os, sys, time, random, pickle

import numpy as np
import pandas as pd

from IPython import embed

current_file_path = os.path.dirname(os.path.realpath(__file__))
sys.path.append(os.path.join(current_file_path, '..', '..'))

from scripts.utils import build_gam

DOMAIN_PATH = os.path.join(current_file_path, '..', '..', '..', 'domains', 'CDB')
FEEDBACK_PATH = os.path.join(DOMAIN_PATH, 'feedback')
PARAM_PATH = os.path.join(DOMAIN_PATH, 'params')

def build_gams():
    """
        params:
            None

        returns:
            None

        description:
            Builds the gam and gam map for both the 'open' and 'cross' actions.
            If more flagged actions are added, there are probably cleaner ways to
            do this.
    """

    # Build and save the gam and gam_map for the action 'open'
    open_gam, open_gam_map = build_gam(pd.read_csv(os.path.join(FEEDBACK_PATH, 'open.data')))

    gam_map_file = open(os.path.join(PARAM_PATH, 'open_gam_map.pkl'), mode = 'wb')
    pickle.dump(open_gam_map, gam_map_file, protocol=pickle.HIGHEST_PROTOCOL)
    gam_map_file.close()
    
    gam_file = open(os.path.join(PARAM_PATH, 'open_gam.pkl'), mode = 'wb')
    pickle.dump(open_gam, gam_file, protocol=pickle. HIGHEST_PROTOCOL)
    gam_file.close()

    # Build and save the gam and gam_map for the action 'cross'
    cross_gam, cross_gam_map = build_gam(pd.read_csv(os.path.join(FEEDBACK_PATH, 'cross.data')))

    gam_map_file = open(os.path.join(PARAM_PATH, 'cross_gam_map.pkl'), mode = 'wb')
    pickle.dump(cross_gam_map, gam_map_file, protocol=pickle.HIGHEST_PROTOCOL)
    gam_map_file.close()

    gam_file = open(os.path.join(PARAM_PATH, 'cross_gam.pkl'), mode = 'wb')
    pickle.dump(cross_gam, gam_file, protocol=pickle. HIGHEST_PROTOCOL)
    gam_file.close()


def load_gams():
    """
        params:
            None

        returns:
            The gams and gam maps for the cross and open actions.

        notes:
            Pickled objects must be store in the PARAM directory to be loaded properly.
    """
    cross_GAM = pickle.load( open( os.path.join(PARAM_PATH, 'cross_gam.pkl'), mode='rb'), encoding='bytes')
    open_GAM = pickle.load( open( os.path.join(PARAM_PATH, 'open_gam.pkl'), mode='rb'), encoding='bytes')

    cross_GAM_map = pickle.load( open( os.path.join(PARAM_PATH, 'cross_gam_map.pkl'), mode='rb'), encoding='bytes')
    open_GAM_map = pickle.load( open( os.path.join(PARAM_PATH, 'open_gam_map.pkl'), mode='rb'), encoding='bytes')

    return cross_GAM, open_GAM, cross_GAM_map, open_GAM_map

class CampusDeliveryBotHelper():
    def __init__(self, DM):
        self.DM = DM
        build_gams()
        self.cross_GAM, self.open_GAM, self.cross_GAM_map, self.open_GAM_map = load_gams()

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
        regions = self.DM.regions
        f1 = regions[(state[0], state[1])]
        f = [f1] + list(state[3:])
        return f

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

    def get_door_type(self, state):
        """
            params:
                state - The state we are looking up the door type for.

            returns:
                The type of door in state or None if state has no door.
        """
        x, y = state[0], state[1]
        if self.DM.grid[x][y] == 'L':
            return 'light'
        elif self.DM.grid[x][y] == 'M':
            return 'medium'
        elif self.DM.grid[x][y] == 'H':
            return 'heavy'
        else:
            return None

    def get_visibility(self, state):
        """
            params:
                state - The state we are looking up the visibility condition for.

            returns:
                The visibility condition in state or None if state is not a crosswalk.
        """
        x, y = state[0], state[1]
        if self.DM.grid[x][y] == 'c':
            return 'low'
        elif self.DM.grid[x][y] == 'C':
            return 'high'
        else:
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

        with open( os.path.join(PARAM_PATH, 'used_features.txt'), mode='a+') as f:
            f.write("," + str(feature))


        df = pd.read_csv( open(os.path.join(FEEDBACK_PATH, action+'.data')))
        df_full = pd.read_csv( open(os.path.join(FEEDBACK_PATH, action+'_full.data')))

        X = df.drop('feedback', axis = 1)
        X.index = range(len(X))
        X[feature] = df_full[feature]
        X['feedback'] = df_full['feedback']

        X.to_csv(os.path.join(FEEDBACK_PATH, action+'.data'),index=False)