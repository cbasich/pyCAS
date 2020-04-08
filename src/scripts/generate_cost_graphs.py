import os, sys

import numpy as np

from matplotlib import pyplot as plt

OUTPUT_PATH = os.path.join('..','..','output','CDB')

def generate_cost_graphs():
    cost_file = open(os.path.join(OUTPUT_PATH,'map_1_costs.txt'), mode = 'r+')
    expected_cost_file = open(os.path.join(OUTPUT_PATH, 'map_1_expected_costs.txt'), mode = 'r+')

    costs, expected_costs = [], []

    for line in cost_file.readlines():
        costs.append(float(line))

    for line in expected_cost_file.readlines():
        expected_costs.append(float(line))

    diffs = 100.0 * ((np.array(costs) - np.array(expected_costs))/np.array(expected_costs))

    fig, ax1 = plt.subplots()
    ax1.set_xlabel('Episode', fontsize=14)
    ax1.set_ylabel('Percent Error in Cost Prediction', fontsize=14)

    ax1.plot(diffs, color = 'steelblue')
    ax1.tick_params(axis='y')

    fig.tight_layout()
    plt.grid(False)

    filepath = os.path.join(OUTPUT_PATH, 'cost_difference_graph.png')
    plt.savefig(filepath)
    plt.clf()

    plt.close(fig)

    cost_file.close()
    expected_cost_file.close()

def generate_candidate_count_graphs():
    candidate_count_file = open(os.path.join(OUTPUT_PATH,'candidate_count.txt'), mode='r+')
    init_state_candidate_count_file = open(os.path.join(OUTPUT_PATH, 'init_state_candidate_count.txt'), mode = 'r+')

    fig, ax1 = plt.subplots()
    ax1.set_xlabel('Episode', fontsize=14)
    ax1.set_ylabel('Number of Candidates',fontsize=14)
    ax1.plot(candidates, color = 'steelblue', label = 'All States')
    ax1.plot(init_state_candidates, color = 'darkkhaki', label = 'Initial States')
    ax1.tick_params(axis = 'y')
    fig.tight_layout()
    plt.grid(False)
    filepath = os.path.join(OUTPUT_PATH, 'candidates_graph.png')
    plt.savefig(filepath)
    plt.clf()

    plt.close(fig)

if __name__ == '__main__':
    generate_cost_graphs()