import numpy as np

def value_iteration(mdp, eps = 0.0001):
    states, actions = list(mdp.states), list(mdp.actions)
    R, T, gamma, = -1.0*np.array(mdp.costs), np.array(mdp.transitions), mdp.gamma

    V = np.array([0.0 for s in range(len(states))]).reshape(-1, 1)
    Q = np.array([[0.0 for a in range(len(actions))] for s in range(len(states))])

    dim_array = np.ones((1, T.ndim), int).ravel()
    dim_array[2] = -1

    while True:
        V_copy = V.copy()
        Q = R + gamma*( np.sum( T * V.reshape(dim_array), axis = 2) )

        V = np.amax(Q, axis = 1)
        if np.max( abs(V - V_copy) ) < eps:
            break

    V *= -1.0

    v = {s: V[s] for s in range(len(states))}
    pi = {s: actions[np.argmax(Q[s])] for s in range(len(states))}
    state_map = {state: s for s, state in enumerate(states)}

    mdp_info = {
        'V': v,
        'pi': pi,
        'state map': state_map,
        'Q' : Q
    }

    return mdp_info