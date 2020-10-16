import os, sys, pickle

import numpy as np
import pandas as pd

from IPython import embed

from pygam.terms import Term, TermList
from pygam import GAM, te, s, f, l

def FVI(mdp, eps = 0.001):
    """
        This is a fast value iteration using vectorized operations only.
        This function can work on any MDP although it assumes cost-minimizing
        rather than reward-maximizing.
        Additionally, this function requires that C/R and T are given as np arrays
        rather than functions.
    """
    states, actions = list(mdp.states), list(mdp.actions)
    R, T, gamma = -1.0*np.array(mdp.costs).astype('float32'), np.array(mdp.transitions).astype('float32'), mdp.gamma

    V = np.zeros((len(states))).astype('float32').reshape(-1,1)
    Q = np.zeros((len(states), len(actions))).astype('float32')

    dim_array = np.ones((1, T.ndim), int).ravel()
    dim_array[2] = -1

    while True:
        Q = R + gamma*( np.sum( T * V.reshape(dim_array), axis = 2) )
        tmp = np.amax(Q, axis = 1)
        if np.max( abs(tmp - V) ) < eps:
            V = tmp
            break
        V = tmp
    V *= -1.0

    v = {s: V[s] for s in range(len(states))}
    pi = {s: actions[np.argmax(Q[s])] for s in range(len(states))}
    state_map = {state: s for s, state in enumerate(states)}

    results = {
        'V': V,
        'pi': pi,
        'state map': state_map,
        'Q' : Q
    }

    return results


def build_gam(df, distr='binomial', link='logit', fast=False, input_classifier=None):
    """
        This function is for building a GAM classifier.
        
        params:
            domain - A string of the name of the domain, e.g. 'CDB'
            name - A string of the name for the classifier, generally the action in question, e.g. 'cross'
            distr - The distribution used for the GAM, defaulted to binomial.
            link - The link used for the GAM, deefaulted to logit.
            input_classifier - If None, will default to a GAM with a splice term on all singleton terms
                               and a tensor term on all pairs of terms. Else will use what is provided.

        returns:
            None

        description:
            This function produces and pickles a GAM classifier on the data provided, as well as a 'gam map'
            which provides the mapping for semantic feature value names used in the data set to the tokenized
            values that the GAM requires.

    """
    # First get all of the features (Xv) and convert into the dataframe identifiers
    Xv = df.values[:,:-1]
    X = np.unique(df.values[:,0:1], return_inverse=True)[1].reshape(-1,1)
    for i in range(1, len(df.values[0])-1):
        if isinstance(df.values[:,i:i+1][0][0], float):
            X = np.concatenate((X, df.values[:, i:i+1].reshape(-1,1)), axis=1)
        else:
            X = np.concatenate((X, np.unique(df.values[:,i:i+1], return_inverse=True)[1].reshape(-1,1) ), axis=1)
    # Second get the labels and convert to the dataframe identifiers
    y = np.unique(df.values[:,-1], return_inverse=True)[1]

    gam_map = {}                        # We will need this for the model. This let's us map semantics into
                                        # dataframe identifiers for calling predict.
    for i in range(df.shape[1]-1):      # Iterate through the number of features
        for key in np.unique(Xv[:,i]):  # Iterate through each value of feature
            try:                        # Try to add the key if it doesn't exist yet
                gam_map[key] = X[:,i][np.where(Xv[:,i] == key)[0]][0]
            except Exception:           # Deal with shaping issues
                X = X.reshape(-1,1)
                gam_map[key] = X[:,i][np.where(Xv[:,i] == key)[0]][0]
    terms = s(0)
    for i in range(df.shape[1]-1):
        if i != 0:
            terms += s(i)
        for j in range(i+1, df.shape[1]-1):
            terms += te(i,j)

    # Build the GAM now. By default use a Logistic GAM.
    # Use gridsearch to determine optimal parameters.
    if fast == True:
        gam = GAM(terms, distribution=distr, link=link).fit(X,y)
    else:
        try:
            gam = GAM(terms, distribution=distr, link=link).gridsearch(X,y)
        except Exception:
            embed()
            return None, gam_map
            # gam = GAM(terms, distribution=distr, link=link).fit(X,y)
    return gam, gam_map