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


def save_agent(agent, training_method):
    if training_method is None:
        training_method = 'None'
    with open(os.path.join(AGENT_PATH, training_method, 'agent_{}.pkl'.format(agent.id)), mode='rb') as f:
        pickle.dump(agent, f, protocol=pickle.HIGHEST_PROTOCOL)

def rollout(info):
    agent_id = info['agent_id']
    training_method = info['training_method']
    with open(os.path.join(AGENT_PATH, training_method, 'agent_{}.pkl'.format(agent_id)), mode='rb') as f:
        agent = pickle.load(f, encoding='bytes')
    agent.CAS.DM.set_random_task()
    agent.CAS.HM.generate_T_H()
    agent.CAS.reset()
    agent.CAS.solve()
    output = agent.rollout()
    return output


def train(info):
    agent_id = info['agent_id']
    training_method = info['training_method']
    with open(os.path.join(AGENT_PATH, training_method, 'agent_{}.pkl'.format(agent_id)), mode='rb') as f:
        agent = pickle.load(f, encoding='bytes')
    agent.CAS.HM.train(agent.id, 'open', training_method)
    agent.CAS.HM.train(agent.id, 'cross', training_method)
    agent.CAS.HM.update_lambda()
    agent.CAS.update_kappa()
    save_agent(agent, training_method)
    # agent.save()


def main(n_agents, num_episodes, training_method='all', frequency=10, n_jobs=4):
    if training_method == 'None':
        training_method = None
    agent_ids = []
    for i in range(n_agents):
        if os.path.exists(os.path.join(AGENT_PATH, training_method, 'agent_{}.pkl'.format(i))):
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


    if training_method == 'all':
        for training_method in [None, 'naive', 'soft_labeling', 'multi_task', 'multi_source']:
            results = []
            for i in range(num_episodes):
                print("Starting episode {}".format(i))
                with mp.Pool(n_jobs) as pool:
                    results.append(pool.map(rollout, [{'agent_id': agent_id, 'training_method': training_method} for agent_id in agent_ids]))
                    if i % frequency == 0 and i != 0:
                        pool.map(train, [{'agent_id': agent_id, 'training_method': training_method} for agent_id in agent_ids])
            with open(os.path.join(OUTPUT_PATH, training_method, 'output.txt'), mode='a+') as f:
                f.write(results)

    else:
        results = []
        for i in range(num_episodes):
            print("Starting episode {}".format(i))
            with mp.Pool(n_jobs) as pool:
                results.append(pool.map(rollout, [{'agent_id': agent_id, 'training_method': training_method} for agent_id in agent_ids]))
                if i % frequency == 0 and i != 0:
                    pool.map(train, [{'agent_id': agent_id, 'training_method': training_method} for agent_id in agent_ids])

        with open(os.path.join(OUTPUT_PATH, training_method, 'output.txt'), mode='a+') as f:
            f.write(results)


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('-m', '--map_file', type=str, default='large_campus.txt')
    parser.add_argument('-n', '--num_agents', type=int, default=4)
    parser.add_argument('-t', '--training_method', type=str, default='None')
    args = parser.parse_args()

    main(4, 301)