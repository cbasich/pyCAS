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


def rollout(agent_id):
    with open(os.path.join(AGENT_PATH, 'agent_{}.pkl'.format(agent_id)), mode='rb') as f:
        agent = pickle.load(f, encoding='bytes')
    agent.CAS.DM.set_random_task()
    agent.CAS.HM.generate_T_H()
    agent.CAS.reset()
    agent.CAS.solve()
    output = agent.rollout()
    agent.save()
    return output


def train(info):
    agent_id = info['agent_id']
    training_method = info['training_method']
    with open(os.path.join(AGENT_PATH, 'agent_{}.pkl'.format(agent_id)), mode='rb') as f:
        agent = pickle.load(f, encoding='bytes')
    agent.CAS.HM.train(agent.id, 'open', training_method)
    agent.CAS.HM.train(agent.id, 'cross', training_method)
    agent.CAS.HM.update_lambda()
    agent.CAS.update_kappa()
    # agent.CAS.save_kappa()
    agent.save()


def main(n_agents, num_episodes, training_method='soft_labeling', frequency=10, n_jobs=4):
    agent_ids = []
    for i in range(n_agents):
        if os.path.exists(os.path.join(AGENT_PATH, 'agent_{}.pkl'.format(i))):
            agent_ids.append(i)
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
            agent_ids.append(Alfred.id)

    # embed()
    # quit()

    results = []
    for i in range(num_episodes):
        print("Starting episode {}\n".format(i))
        with mp.Pool(n_jobs) as pool:
            results.append(pool.map(rollout, agent_ids))
            if i % frequency == 0 and i != 0:
                pool.map(train, [{'agent_id': agent_id, 'training_method': training_method} for agent_id in agent_ids])
        print(results[-1:])
    print(results)

    embed()

if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('-m', '--map_file', type=str, default='large_campus.txt')
    parser.add_argument('-n', '--num_agents', type=int, default=4)
    args = parser.parse_args()

    main(4, 11)