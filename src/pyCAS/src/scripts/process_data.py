import os, sys, pickle, argparse

import numpy as np
import pandas as pd

from IPython import embed

def get_reachable_states(policies, CAS):
    reachable_states = {}

    for i in list(policies.keys()):
        pi = policies[i]['policy']
        state_map = policies[i]['state_map']

        states = set()
        queued = [CAS.init]

        while len(queued) > 0:
            state = queued.pop(0)
            states.add(state)
            try:
                action = pi[state_map[state]]
            except Exception:
                tmp = (state[0][:-1], state[1])
                action = pi[state_map[tmp]]

            Tsa = CAS.transitions[CA-S.states.index(state)][CAS.actions.index(action)]
            for sp in range(len(Tsa)):
                if Tsa[sp] > 0.0 and CAS.states[sp] != state:
                    queued.append(CAS.states[sp])

        reachable_states[i] = states

    return reachable_states

def get_visited_states(execution_trace_file_path):
    f = open(execution_trace_file_path, 'r+')

    visited_states = []

    for line in f.readlines():
        if 'BEGINNING' in line or 'Feedback' in line or 'Discriminator' in line:
            continue

        line_split = line.split('|')
        state = line_split[0][1:-2]
        state_info = state[1:-4]
        state_level = int(state[-1:])
        state_info = state_info.split(', ')

        for i in range(len(state_info)):
            if state_info[i][0] == "'":
                state_info[i] = state_info[i][1:-1]         # Remove the single quotes, but leave as a string
            else:
                state_info[i] = int(state_info[i])          # Otherwise it is a location so cast to an int

        state_info = tuple(state_info)

        visited_states.append((state_info, state_level))    # Append info and level back together and add as one state

    return visited_states
