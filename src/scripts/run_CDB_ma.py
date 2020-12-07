import os, sys, time, json, random, copy, pickle, argparse

from collections import defaultdict
from IPython import embed
from matplotlib import pyplot as plt

import numpy as np
import pandas as pd
import itertools as it
import multiprocessing as mp

current_file_path = os.path.dirname(os.path.realpath(__file__))
sys.path.append(os.path.join(current_file_path, '..'))

import utils
import process_data

from models.CDB_ma import autonomy_model, feedback_model, domain_model, competence_aware_system, agent

OUTPUT_PATH = os.path.join(current_file_path, '..', '..', 'output', 'CDB_ma')
AGENT_PATH = os.path.join(current_file_path, '..', '..', 'domains', 'CDB_ma', 'agents')
MAP_PATH = os.path.join(current_file_path, '..', '..', 'domains', 'CDB_ma', 'maps')


def rollout(agent):
    agent.CAS.DM.set_random_task()
    agent.CAS.HM.generate_T_H()
    agent.CAS.reset()
    agent.CAS.solve()
    return agent.rollout()


def train(info):
    agent = info['agent']
    training_method = info['training_method']
    agent.CAS.HM.train(agent.id, 'open', training_method)
    agent.CAS.HM.train(agent.id, 'cross', training_method)


def main(n_agents, num_episodes, training_method='multi_task', frequency=1, n_jobs=4):
    agents = []
    for i in range(n_agents):
        if os.path.exists(os.path.join(AGENT_PATH, 'agent_{}'.format(i))):
            with open(os.path.join(AGENT_PATH, 'agent_{}'.format(i))) as f:
                agents.append(pickle.load(f, encoding='bytes'))
        else:
            Alfred = agent.Agent(i)
            DM = domain_model.DomainModel('small_campus.txt')
            AM = autonomy_model.AutonomyModel(DM, L=[0,1,2,3])
            HM = feedback_model.FeedbackModel(DM, AM, Sigma=['+', '-', '/', None],
                flagged_actions=['open', 'cross'], training_method = None)
            
            HM.train(Alfred.id, 'open', None)
            HM.train(Alfred.id, 'cross', None)
            HM.generate_lambda()
            
            CAS = competence_aware_system.CAS(DM, AM, HM, persistence=0.75)
            Alfred.set_cas_model(CAS)
            Alfred.set_authority_epsilon((i+1) * 0.02)
            Alfred.save()
            agents.append(Alfred)

    # embed()
    # quit()

    results = []
    for i in range(num_episodes):
        print("Starting episode {}\n".format(i))
        with mp.Pool(n_jobs) as pool:
            results.append(pool.map(rollout, agents))
            if i % frequency == 0 and i != 0:
                pool.map(train, [{'agent': agent, 'training_method': training_method} for agent in agents])
    print(results)

if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('-m', '--map_file', type=str, default='large_campus.txt')
    parser.add_argument('-n', '--num_agents', type=int, default=4)
    args = parser.parse_args()

    main(4,21)