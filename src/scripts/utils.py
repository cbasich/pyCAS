import os, sys, pickle

import numpy as np
import pandas as pd

from IPython import embed

from pygam.terms import Term, TermList
from pygam import GAM, te, s, f, l

from sklearn import svm
from sklearn.pipeline import make_pipeline
from sklearn.preprocessing import OneHotEncoder, normalize, StandardScaler

from scipy.stats import entropy

current_file_path = os.path.dirname(os.path.realpath(__file__))
sys.path.append(os.path.join(current_file_path, '..'))
AGENT_PATH = os.path.join(current_file_path, '..', '..', 'domains', 'CDB_ma', 'agents')

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


def _train_model(X, y):
    return svm.SVR(C=1.0, epsilon=0.2).fit(X,y)


def _train_model_naive(agent_id, X, y, action):
    other_id = 0
    while os.path.exists(os.path.join(AGENT_PATH, 'agent_{}.pkl'.format(other_id))):
        if other_id == agent_id:
            other_id += 1
            continue
        with open(os.path.join(AGENT_PATH, 'agent_{}.pkl'.format(other_id)), mode='rb') as f:
            other_agent = pickle.load(f, encoding='bytes')
            other_X, other_y = _load_dataset(action, other_id)
            X = np.concatenate((X, other_X))
            y = np.concatenate((y, other_y.reshape(-1,)))
            other_id += 1

    return svm.SVR(C=1.0, epsilon=0.2).fit(X,y)


def _train_model_soft_labeling(agent_id, target_X, y, action, h):
    other_id = 0
    # y = float(y) - 0.500000
    # embed()
    X_unique = target_X[np.unique(target_X, axis=0, return_index=True)[1]]
    target_weight = np.ones(len(target_X))
    while os.path.exists(os.path.join(AGENT_PATH, 'agent_{}.pkl'.format(other_id))):
        if other_id == agent_id:
            other_id += 1
            continue
        with open(os.path.join(AGENT_PATH, 'agent_{}.pkl'.format(other_id)), mode='rb') as f:
            other_agent = pickle.load(f, encoding='bytes')
        other_X, other_y = _load_dataset(action, other_id)
        target_probs = h.predict(X_unique)
        other_probs = other_agent.CAS.HM.get_classifier(action).predict(X_unique)
        KLD = entropy(target_probs, qk=other_probs)
        y = np.concatenate((y, other_y.reshape(-1,))) #(other_y - 0.500000)))
        target_weight = np.concatenate((target_weight, np.ones(len(other_y)) * KLD))
    return svm.SVR(C=1.0, epsilon=0.2).fit(target_X, y, sample_weight=target_weight)


def _train_model_multi_source(agent_id, target_X, target_y, action, num_iters=10):
    source_datasets = []
    source_weights = []
    target_weight = np.ones(len(target_X))
    n = 0
    other_id = 0
    while os.path.exists(os.path.join(AGENT_PATH, 'agent_{}.pkl'.format(other_id))):
        if other_id == agent_id:
            other_id += 1
            continue
        with open(os.path.join(AGENT_PATH, 'agent_{}.pkl'.format(other_id)), mode='rb') as f:
            other_agent = pickle.load(f, encoding='bytes')
            other_X, other_y = _load_dataset(action, other_id)
        source_datasets.append((other_X, other_y))
        n += len(other_X)
        source_weights.append(np.ones(len(other_y)))
        other_id += 1
    alpha_S = 0.5 * np.log(1 + np.sqrt(2 * np.log(n / num_iters)))

    for t in range(num_iters):
        weak_classifiers = set()
        target_weight /= np.sum(target_weight)
        for weight in source_weights:
            weight /= np.sum(weight)
        for k in range(len(source_datasets)):
            combined_X = np.concatenate((source_datasets[k][0], target_X))
            combined_y = np.concatenate((source_datasets[k][1].reshape(-1,), target_y))
            combined_weight = np.concatenate((source_weights[k], target_weight))
            weak_classifier = svm.SVC().fit(combined_X, combined_y, sample_weight=combined_weight)
            y_pred = weak_classifier.predict(target_X)
            error = np.dot(target_weight, target_y == y_pred) / np.sum(target_weight)
            weak_classifiers.add((weak_classifier, error))
        best_classifier = min(weak_classifiers, key=lambda item: item[1])
        alpha_T = 0.5 * np.log((1 - best_classifier[1]) / best_classifier[1])

        # This should maybe only happen for the source that led to best classifier instead of all?
        for k in range(len(source_datasets)):
            y_pred = best_classifier[0].predict(source_datasets[k][0])
            source_weights[k] *= np.exp(-1.0 * alpha_S * (y_pred ^ source_datasets[k][1].reshape(-1,)))

        y_pred = best_classifier[0].predict(target_X)
        target_weight *= np.exp(alpha_T * (y_pred ^ target_y))

    return svm.SVR(C=1.0, epsilon=0.2).fit(target_X, target_y, sample_weight=target_weight)


def _train_model_multi_task(agent_id, target_X, target_y, action, num_iters=10):
    target_weight = np.ones(len(target_X))
    source_classifiers = []
    other_id = 0
    while os.path.exists(os.path.join(AGENT_PATH, 'agent_{}.pkl'.format(other_id))):
        if other_id == agent_id:
            other_id += 1
            continue
        with open(os.path.join(AGENT_PATH, 'agent_{}.pkl'.format(other_id)), mode='rb') as f:
            agent = pickle.load(f, encoding='bytes')
            source_classifiers.append(agent.CAS.HM.get_classifier(action))
        other_id += 1
    for t in range(min(len(source_classifiers), num_iters)):
        if len(source_classifiers) < 1:
            break
        weak_classifiers = set()
        target_weight /= np.sum(target_weight)
        for h in source_classifiers:
            y_pred = h.predict(target_X) > 0.5
            error = np.dot(target_weight, target_y == y_pred)
            # TODO insert the epsilon > 1/2 clause?
            weak_classifiers.add((h, error))
        best_classifier = min(weak_classifiers, key=lambda item: item[1])
        source_classifiers.remove(best_classifier[0])
        alpha_T = 0.5 * np.log((1 - error) / error)
        target_weight *= np.exp(-1.0 * alpha_T * target_y * y_pred)

    return svm.SVR(C=1.0, epsilon=0.2).fit(target_X, target_y, sample_weight=target_weight)


def _load_dataset(action, other_id):
    agent = None
    with open(os.path.join(AGENT_PATH, 'agent_{}.pkl'.format(other_id)), mode='rb') as f:
        agent = pickle.load(f, encoding='bytes')

    if action == 'open':
        X = agent.CAS.HM.open_enc.transform(agent.CAS.HM.open_data[:,:-1]).toarray()
        y = agent.CAS.HM.open_data[:,-1:] == 'yes'
    elif action == 'cross':
        X = agent.CAS.HM.cross_enc.transform(agent.CAS.HM.cross_data[:,:-1]).toarray()
        y = agent.CAS.HM.cross_data[:,-1:] == 'yes'

    return X, y


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
            # embed()
            return None, gam_map
            # gam = GAM(terms, distribution=distr, link=link).fit(X,y)
    return gam, gam_map