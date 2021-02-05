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

OUTPUT_PATH = os.path.join(current_file_path, '..', '..', 'output', 'CDB_ma')


def main():
    none_out = 
    naive_out = 
    multi_source_out = 
    multi_task_out =

    agent_comps_none = [[],[],[],[]]
    for episode_value in none_out:
        for i in range(len(episode_value)):
            agent_comps_none[i].append(episode_value[i][1])

    agent_comps_naive = [[],[],[],[]]
    for episode_value in naive_out:
        for i in range(len(episode_value)):
            agent_comps_naive[i].append(episode_value[i][1])

    agent_comps_multi_source = [[],[],[],[]]
    for episode_value in multi_source_out:
        for i in range(len(episode_value)):
            agent_comps_none[i].append(episode_value[i][1])

    agent_comps_multi_task = [[],[],[],[]]
    for episode_value in multi_task_out:
        for i in range(len(episode_value)):
            agent_comps_none[i].append(episode_value[i][1])

    
    fig, ax = plt.subplots(2, 4, sharex = True, sharey = True)

    axs[0, 0].plot(agent_comps_none[0], label='eps=0.02', color='light blue')
    axs[0, 0].plot(agent_comps_none[1], label='eps=0.04', color='green')
    axs[0, 0].plot(agent_comps_none[2], label='eps=0.06', color='red')
    axs[0, 0].plot(agent_comps_none[3], label='eps=0.08', color='dark blue')
    axs[0, 0].set_title('No FWL')

    axs[0, 1].plot(agent_comps_naive[0], label='eps=0.02', color='light blue')
    axs[0, 1].plot(agent_comps_naive[1], label='eps=0.04', color='green')
    axs[0, 1].plot(agent_comps_naive[2], label='eps=0.06', color='red')
    axs[0, 1].plot(agent_comps_naive[3], label='eps=0.08', color='dark blue')
    axs[0, 1].set_title('Naive FWL')

    axs[0, 2].plot(agent_comps_multi_source[0], label='eps=0.02', color='light blue')
    axs[0, 2].plot(agent_comps_multi_source[1], label='eps=0.04', color='green')
    axs[0, 2].plot(agent_comps_multi_source[2], label='eps=0.06', color='red')
    axs[0, 2].plot(agent_comps_multi_source[3], label='eps=0.08', color='dark blue')
    axs[0, 2].set_title('Multi Source FWL')

    axs[0, 3].plot(agent_comps_multi_task[0], label='eps=0.02', color='light blue')
    axs[0, 3].plot(agent_comps_multi_task[1], label='eps=0.04', color='green')
    axs[0, 3].plot(agent_comps_multi_task[2], label='eps=0.06', color='red')
    axs[0, 3].plot(agent_comps_multi_task[3], label='eps=0.08', color='dark blue')
    axs[0, 3].set_title('Multi Task FWL')

    axs[1, 0].plot(agent_comps_none[0], label='No FWL', color='light blue')
    axs[1, 0].plot(agent_comps_naive[0], label='Naive FWL', color='green')
    axs[1, 0].plot(agent_comps_multi_source[0], label='Multi Source FWL', color='red')
    axs[1, 0].plot(agent_comps_multi_task[0], label='Multi Task FWL', color='dark blue')
    axs[1, 0].set_title('Agent 1, eps=0.02')

    axs[1, 1].plot(agent_comps_none[1], label='No FWL', color='light blue')
    axs[1, 1].plot(agent_comps_naive[1], label='Naive FWL', color='green')
    axs[1, 1].plot(agent_comps_multi_source[1], label='Multi Source FWL', color='red')
    axs[1, 1].plot(agent_comps_multi_task[1], label='Multi Task FWL', color='dark blue')
    axs[1, 1].set_title('Agent 2, eps=0.04')

    axs[1, 2].plot(agent_comps_none[2], label='No FWL', color='light blue')
    axs[1, 2].plot(agent_comps_naive[2], label='Naive FWL', color='green')
    axs[1, 2].plot(agent_comps_multi_source[2], label='Multi Source FWL', color='red')
    axs[1, 2].plot(agent_comps_multi_task[2], label='Multi Task FWL', color='dark blue')
    axs[1, 3].set_title('Agent 3, eps=0.06')

    axs[1, 3].plot(agent_comps_none[3], label='No FWL', color='light blue')
    axs[1, 3].plot(agent_comps_naive[3], label='Naive FWL', color='green')
    axs[1, 3].plot(agent_comps_multi_source[3], label='Multi Source FWL', color='red')
    axs[1, 3].plot(agent_comps_multi_task[3], label='Multi Task FWL', color='dark blue')
    axs[1, 3].set_title('Agent 4, eps=0.08')

if __name__ == '__main__':
    main()
