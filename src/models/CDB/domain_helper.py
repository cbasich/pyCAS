import os, sys, time, random, pickle

import numpy as np

current_file_path = os.path.dirname(os.path.realpath(__file__))
sys.path.append(os.path.join(current_file_path, '..'))

DATA_PATH = os.path.join(current_file_path, '..', 'data')
GAM_DATA_PATH = os.path.join(DATA_PATH, 'GAMS')
CLASSIFIER_DATA_PATH = os.path.join(DATA_PATH, 'classifier')

def build_GAMS():
    build_GAM(os.path.join(CROSS_DATA_PATH, 'cross.data'))
    build_GAM(os.path.join(OPEN_DATA_PATH, 'open.data'))

def load_GAMS():
    cross_GAM = pickle.load( open( os.path.join(GAM_DATA_PATH, 'cross_GAM.pkl'), mode = 'rb'), encoding = 'bytes')
    open_GAM = pickle.load( open( os.path.join(GAM_DATA_PATH, 'open_GAM.pkl'), mode = 'rb'), encoding = 'bytes')

    cross_GAM_map = pickle.load( open( os.path.join(GAM_DATA_PATH, 'cross_GAM_map.pkl'), mode = 'rb'), encoding = 'bytes')
    open_GAM_map = pickle.load( open( os.path.join(GAM_DATA_PATH, 'open_GAM_map.pkl'), mode = 'rb'), encoding = 'bytes')

class CampusDeliveryBotHelper():
    def __init__(self, DM):
        self.DM = DM
        build_GAMS()
        self.cross_GAM, self.open_GAM, self.cross_GAM_map, self.open_GAM_map = load_GAMS()


    def extract_state_features(self, state):
        regions = self.DM.regions
        f1 = regions[(state[0], state[1])]
        f2 = state[3]
        f = [f1, f2]
        return f

    def predict(self, features, action, level):
        if action == 'open':
            return self.open_GAM.predict_proba([[ self.open_GAM_map[f] for f in self.extract_state_features(state)]])

        else:
            try:
                return self.cross_GAM.predict_proba([[ self.cross_GAM_map[f] for f in self.extract_state_features(state)]])
            except Exception:
                return -1