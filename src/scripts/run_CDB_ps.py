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

from models.CDB_ps import autonomy_model, feedback_model, perception_sensitive_mdp, competence_aware_system, agent

OUTPUT_PATH = os.path.join(current_file_path, '..', '..', 'output', 'CDB_ps')
AGENT_PATH = os.path.join(current_file_path, '..', '..', 'domains', 'CDB_ps', 'agents')
MAP_PATH = os.path.join(current_file_path, '..', '..', 'domains', 'CDB_ps', 'maps')


def save_agent(agent):
    with open(os.path.join(AGENT_PATH, 'agent_{}.pkl'.format(agent.id)), mode='wb') as f:
        pickle.dump(agent, f, protocol=pickle.HIGHEST_PROTOCOL)

def rollout(agent_id):
    with open(os.path.join(AGENT_PATH, 'agent_{}.pkl'.format(agent_id)), mode='rb') as f:
        agent = pickle.load(f, encoding='bytes')
    print("Simulating episode for agent {}".format(agent_id))
    agent.CAS.DM.set_random_task()
    agent.CAS.HM.generate_T_H()
    agent.CAS.reset()
    agent.CAS.solve()
    output = agent.rollout()
    save_agent(agent)
    return output


def train(agent_id):
    with open(os.path.join(AGENT_PATH, 'agent_{}.pkl'.format(agent_id)), mode='rb') as f:
        agent = pickle.load(f, encoding='bytes')
    print("Training agent {}".format(agent_id))
    agent.CAS.HM.train(agent.id, 'open', 'none')
    agent.CAS.HM.train(agent.id, 'cross', 'none')
    agent.CAS.HM.update_lambda()
    agent.CAS.DM.set_random_task()
    agent.CAS.HM.generate_T_H()
    agent.CAS.reset()
    agent.CAS.solve()
    agent.CAS.update_kappa()
    save_agent(agent)


def main(num_episodes, map_file, agent_id=0, frequency=10):
    if os.path.exists(os.path.join(AGENT_PATH, 'agent_{}.pkl'.format(i))):
        with open(os.path.join(AGENT_PATH, 'agent_{}.pkl'.format(agent_id)), mode='rb') as f:
            agent = pickle.load(f, encoding='bytes')
    else:
        agent = agent.Agent(i)
        DM = perception_sensitive_mdp.PerceptionSensitiveMDP(map_file)
        AM = autonomy_model.AutonomyModel(DM, L=[0,1,2,3])
        HM = feedback_model.FeedbackModel(DM, AM, Sigma=['+', '-', '/', None],
            flagged_actions=['open', 'cross'], training_method = 'none')
    
        HM.train(agent.id, 'open', 'none')
        HM.train(agent.id, 'cross', 'none')
        HM.generate_lambda()
    
        CAS = competence_aware_system.CAS(DM, AM, HM, persistence=0.75)
        agent.set_cas_model(CAS)
        agent.set_authority_epsilon((i+1) * 0.05)
        save_agent(agent)

    results = []
    for i in range(num_episodes):
        print("Starting episode {}".format(i))
        results.append(rollout(agent.id))
        if (i+1) % frequency == 0:
            print("Training...")
            train(agent.id)


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('-m', '--map_file', type=str, default='small_campus.txt')
    parser.add_argument('-n', '--num_episodes', type=int, default=100)
    parser.add_argument('-f', '--frequency', type=int, default=10)
    args = parser.parse_args()

    main(args.num_agents, args.num_episodes, args.map_file, args.frequency)
