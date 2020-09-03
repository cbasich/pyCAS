import os

import numpy as np
import pickle

from IPython import embed

import CAS_v2

def main():
    policies = pickle.load( open( os.path.join('..', 'pickles', 'policies.pkl'), mode='rb'), encoding='bytes')
    reachable_states = {}

    for i in list(policies.keys()):
        print(i)
        pi = policies[i]['policy']
        state_map = policies[i]['state_map']

        CAS = CAS_v2.ACA('campus_delivery_map_complex.txt', 'd', 's', 0.75)

        states = set()
        queued = [CAS.init]

        while len(queued) > 0:
            state = queued.pop(0)
            states.add(state)
            action = pi[state_map[state]]

            Tsa = CAS.transitions[CAS.states.index(state)][CAS.actions.index(action)]
            for sp in range(len(Tsa)):
                if Tsa[sp] > 0.0 and CAS.states[sp] != state:
                    queued.append(CAS.states[sp])

        reachable_states[i] = states
        pickle.dump(reachable_states, open( os.path.join('..', 'pickles', 'reachable_states.pkl'), mode='wb'), protocol=pickle.HIGHEST_PROTOCOL)

if __name__ == '__main__':
    main()