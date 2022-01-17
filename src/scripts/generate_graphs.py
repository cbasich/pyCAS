import os, sys,  pickle

import numpy as np

from matplotlib import pyplot as plt
from IPython import embed

OUTPUT_PATH = os.path.join('..','..','output','CDB_aij')

def get_smoothed_values(data, window_size):
    smoothed_values = []
    for i in range(1, len(data)):
        truncated_data = data[:i]
        windowed_data = truncated_data[-window_size:]
        smoothed_values.append(np.average(windowed_data))
    # println(smoothed_values)
    return smoothed_values

def generate_cost_graphs():
    cost_file = open(os.path.join(OUTPUT_PATH, 'update', 'large_campus_costs.txt'), mode = 'r+')
    expected_cost_file = open(os.path.join(OUTPUT_PATH, 'update', 'large_campus_expected_costs.txt'), mode = 'r+')

    costs, expected_costs = [], []

    for line in cost_file.readlines():
        tmp = [float(x) for x in line.split(',')[:-1]]
        costs.append(tmp)

    for line in expected_cost_file.readlines():
        expected_costs.append(float(line))

    costs = np.array(costs)
    expected_costs = np.array(expected_costs).reshape(-1,1)

    diffs = 100 * (costs - expected_costs)/expected_costs
    avgs = get_smoothed_values(np.mean(diffs, axis = 1), 5)
    stes = get_smoothed_values(np.sqrt(np.std(diffs, axis = 1)), 5)

    fig, ax1 = plt.subplots()
    ax1.set_xlabel('Episode', fontsize=14)
    ax1.set_ylabel('Percent Error in Cost Prediction', fontsize=14)

    ax1.errorbar(x = np.arange(200), y = avgs[:200], yerr = stes[:200], color='steelblue', ecolor='orange')
    ax1.tick_params(axis='y')

    fig.tight_layout()
    plt.grid(False)

    filepath = os.path.join(OUTPUT_PATH, 'cost_difference_graph.png')
    plt.savefig(filepath)
    plt.clf()

    plt.close(fig)

    cost_file.close()
    expected_cost_file.close()

def generate_competence_graphs():
    alo_vanilla, alo_update, vlo_vanilla, vlo_update, fc_vanilla, fc_update = [], [], [], [], [], []
    with open(os.path.join(OUTPUT_PATH, 'vanilla', 'large_campus_alo.txt'), mode = 'r+') as f:
        alo_vanilla = get_smoothed_values(np.array([float(x) for x in f.readline().split(',')]), 5)
    with open(os.path.join(OUTPUT_PATH, 'update', 'large_campus_alo.txt'), mode = 'r+') as f:
        alo_update = get_smoothed_values(np.array([float(x) for x in f.readline().split(',')]), 5)

    with open(os.path.join(OUTPUT_PATH, 'update', 'large_campus_fbc.txt'), mode = 'r+') as f:
        fc_update = np.array(np.array([int(x) for x in f.readline().split(',')]))

    with open(os.path.join(OUTPUT_PATH, 'vanilla', 'competence_graph_info.pkl'), mode = 'rb') as f:
        tmp = pickle.load(f, encoding='bytes')
        vlo_vanilla = get_smoothed_values(tmp['visited_LO'], 5)
        fc_vanilla = tmp['feedback_count']
    with open(os.path.join(OUTPUT_PATH, 'update', 'competence_graph_info.pkl'), mode = 'rb') as f:
        tmp = pickle.load(f, encoding='bytes')
        vlo_update = get_smoothed_values(tmp['visited_LO'], 5)
        # fc_update = tmp['feedback_count']

    fig, ax1 = plt.subplots()
    ax1.set_xlabel('Episode', fontsize=14)
    ax1.set_ylabel('% Policy at Competence', fontsize=14)
    ax1.set_ylim(top=100)

    ax1.plot(alo_vanilla[:200], color = 'skyblue', label = 'All States Vanilla')
    ax1.plot(alo_update[:200], color = 'steelblue', label = 'All States Modifed')
    ax1.plot(vlo_vanilla[:200], color = 'pink', label = 'Visited States Vanilla')
    ax1.plot(vlo_update[:200], color = 'indianred', label = 'Visited States Modifed')
    ax1.tick_params(axis='y')

    ax2 = ax1.twinx()
    ax2.set_ylabel('Number of Feedback Signals', fontsize=14)
    ax2.plot(fc_update[:200], color = 'black', label = 'Cumulative Signals Modified')
    ax2.plot(fc_vanilla[:200], color = 'grey', label = 'Cumulative Signals Vanilla')
    ax2.tick_params(axis = 'y')

    fig.tight_layout()
    plt.grid(False)

    lines, labels = ax1.get_legend_handles_labels()
    lines2, labels2 = ax2.get_legend_handles_labels()
    ax1.legend(lines + lines2, labels + labels2, loc=4, fontsize=14)

    filepath = os.path.join(OUTPUT_PATH, 'competence_graph.png')
    plt.savefig(filepath)
    plt.clf()
    plt.close(fig)

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
    generate_competence_graphs()
