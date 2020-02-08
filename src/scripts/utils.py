import os, sys, pickle

import numpy as np
import pandas as pd

from pygam import LogisticGAM, te, s, f, l

def FVI(mdp, eps = 0.001):
    """
        This is a fast value iteration using vectorized operations only.
        This function can work on any MDP although it assumes cost-minimizing
        rather than reward-maximizing.
        Additionally, this function requires that C/R and T are given as np arrays
        rather than functions.
    """
    states, actions = list(mdp.states), list(mdp.actions)
    R, T, gamma, = -1.0*np.array(mdp.costs), np.array(mdp.transitions), mdp.gamma

    V = np.array([0.0 for s in range(len(states))]).reshape(-1, 1)
    Q = np.array([[0.0 for a in range(len(actions))] for s in range(len(states))])

    dim_array = np.ones((1, T.ndim), int).ravel()
    dim_array[2] = -1

    while True:
        # V_copy = V.copy()
        Q = R + gamma*( np.sum( T * V.reshape(dim_array), axis = 2) )
        tmp = np.amax(Q, axis = 1)
        if np.max( abs(tmp - V) ) < eps:
            V = tmp
            break
        V = tmp

        # V = np.amax(Q, axis = 1)
        # if np.max( abs(V - V_copy) ) < eps:
        #     break

    V *= -1.0

    v = {s: V[s] for s in range(len(states))}
    pi = {s: actions[np.argmax(Q[s])] for s in range(len(states))}
    state_map = {state: s for s, state in enumerate(states)}

    results = {
        'V': v,
        'pi': pi,
        'state map': state_map,
        'Q' : Q
    }

    return results

def build_gam(domain, name, distr='binomial', link='logit'):
    """
        This function is for building a GAM classifier.
        It requires a pandas dataframe and assumes no missing values.
    """
    domain_path = os.path.join('..', '..', 'domains', domain)
    df = pd.read_csv(os.path.join(domain_path, 'feedback', name+'.data'))

    # First get all of the features (Xv) and convert into the dataframe identifiers
    Xv = df.values[:,:-1]
    X = np.unique(df.values[:,0:1], return_inverse=True)[1].reshape(-1,1)
    for i in range(1, len(df.values[0])-1):
        X = np.concatenate((X, np.unique(df.values[:,i:i+1], return_inverse=True)[1].reshape(-1,1) ), axis=1)

    # Second get the labels and convert to the dataframe identifiers
    y = np.unique(df.values[:,-1], return_inverse=True)[1]

    gam_map = {}                # We will need this for the model. This let's us map semantics into
                                # dataframe identifiers for calling predict.
    for i in range(df.shape[1]-1):      # Iterate through the number of features
        for key in np.unique(Xv[:,i]):  # Iterate through each value of feature
            try:                # Try to add the key if it doesn't exist yet
                gam_map[key] = X[:,i][np.where(Xv[:,i] == key)[0]][0]
            except Exception:   # Deal with shaping issues
                X = X.reshape(-1,1)
                gam_map[key] = X[:,i][np.where(Xv[:,i] == key)[0]][0]

    gam_map_file = open(os.path.join(domain_path, 'params', name + '_gam_map.pkl'), mode = 'wb')
    pickle.dump(gam_map, gam_map_file, protocol=pickle.HIGHEST_PROTOCOL)
    gam_map_file.close()

    # Build the GAM now. By default use a Logistic GAM.
    # Use gridsearch to determine optimal parameters.
    # TODO: See if there is a way to automatically use both spline and tensor features.
    #       If not - will need to manually encode that since it is not in it right now.
    # gam = LogisticGAM().gridsearch(X, y)
    gam = LogisticGAM().fit(X,y)

    gam_file = open(os.path.join(domain_path, 'params', name + '_gam.pkl'), mode = 'wb')
    pickle.dump(gam, gam_file, protocol=pickle. HIGHEST_PROTOCOL)
    gam_file.close()