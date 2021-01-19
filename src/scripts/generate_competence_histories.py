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


def main(training_method):
    if training_method == 'all':
        for method in ['none', 'naive', 'multi_source', 'multi_task']:
            competence_values = {}
            agent_id = 0
            while os.path.exists(os.path.join(AGENT_PATH, method, 'agent_{}.pkl'.format(agent_id))):
                with open(os.path.join(AGENT_PATH, method, 'agent_{}.pkl'.format(agent_id)), mode='rb') as f:
                    agent = pickle.load(f, encoding='bytes')
                    competence_history = agent.competence_history
                    competence_values[i] = competence_history
            with open(os.path.join(OUTPUT_PATH, method, 'competence_history.json)', mode='w+')) as f:
                json.dump(competence_values, f, indent=4)
    else:
        competence_values = {}
        agent_id = 0
        while os.path.exists(os.path.join(AGENT_PATH, training_method, 'agent_{}.pkl'.format(agent_id))):
            with open(os.path.join(AGENT_PATH, training_method, 'agent_{}.pkl'.format(agent_id)), mode='rb') as f:
                agent = pickle.load(f, encoding='bytes')
                competence_history = agent.competence_history
                competence_values[i] = competence_history
        with open(os.path.join(OUTPUT_PATH, training_method, 'competence_history.json)', mode='w+')) as f:
            json.dump(competence_values, f, indent=4)


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('-t', '--training_method', type=str, default='all')
    args = parser.parse_args()

    main(args.training_method)